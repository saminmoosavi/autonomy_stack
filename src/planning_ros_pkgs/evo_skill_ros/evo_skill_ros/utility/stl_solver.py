# stl_solver.py
import numpy as np
from stlpy.STL import LinearPredicate
from stlpy.systems import NonlinearSystem
from stlpy.solvers import DrakeSmoothSolver

def make_unicycle_context_system(dt, cone_xy, goal_xy, distance_tolerance, cone_radius=0.15, goal_radius=0.25):
    cx, cy = cone_xy
    gx, gy = goal_xy
    safe_radius = cone_radius + distance_tolerance

    def f(x, u):
        px, py, th = x
        v, om = u
        return np.array([px + dt * v * np.cos(th), py + dt * v * np.sin(th), th + dt * om])

    def g(x, u):
        px, py, th = x
        cone_clearance = (px - cx)**2 + (py - cy)**2 - safe_radius**2
        goal_margin = goal_radius**2 - ((px - gx)**2 + (py - gy)**2)
        return np.array([px, py, th, cone_clearance, goal_margin])

    return NonlinearSystem(f=f, g=g, n=3, m=2, p=5)

def make_spec(T):
    cone_safe = LinearPredicate(a=np.array([0.0, 0.0, 0.0, 1.0, 0.0]), b=0.0)
    at_goal = LinearPredicate(a=np.array([0.0, 0.0, 0.0, 0.0, 1.0]), b=0.0)
    return cone_safe.always(0, T) & at_goal.eventually(0, T)

def compute_stl_trajectory(x0, goal_xy, cone_xy, distance_tolerance=0.45, T=60, dt=0.1):
    """
    Standalone function called by the ROS 2 node.
    Returns: x_roll (3, T+1) trajectory array of [x, y, theta]
    """
    goal_xy = np.asarray(goal_xy, dtype=float)
    cone_xy = np.asarray(cone_xy, dtype=float)
    sys = make_unicycle_context_system(dt, cone_xy, goal_xy, distance_tolerance)
    phi = make_spec(T)

    solver = DrakeSmoothSolver(spec=phi, sys=sys, x0=x0.reshape(-1, 1), T=T, k=10.0, verbose=False)
    solver.AddControlBounds(np.array([0.0, -1.5]), np.array([1.0, 1.5]))
    xy_points = np.vstack([x0[:2], goal_xy, cone_xy])
    xy_min = np.min(xy_points, axis=0) - 2.0
    xy_max = np.max(xy_points, axis=0) + 2.0
    solver.AddStateBounds(
        np.array([xy_min[0], xy_min[1], -2*np.pi]),
        np.array([xy_max[0], xy_max[1], 2*np.pi]),
    )
    solver.AddQuadraticCost(0.01 * np.eye(3), np.diag([0.05, 0.05]))
    solver.AddRobustnessConstraint(rho_min=0.0)

    print(
        "[stl_solver] Calling Drake STL solver: "
        f"start=({x0[0]:.3f}, {x0[1]:.3f}, {x0[2]:.3f}), "
        f"goal=({goal_xy[0]:.3f}, {goal_xy[1]:.3f}), "
        f"obstacle=({cone_xy[0]:.3f}, {cone_xy[1]:.3f}), "
        f"T={T}, dt={dt}, "
        f"state_bounds=([{xy_min[0]:.3f}, {xy_min[1]:.3f}, {-2*np.pi:.3f}], "
        f"[{xy_max[0]:.3f}, {xy_max[1]:.3f}, {2*np.pi:.3f}])",
        flush=True,
    )
    x_opt, u_opt, _, _ = solver.Solve()
    if x_opt is None or u_opt is None:
        raise RuntimeError("Drake STL solver did not return a trajectory")

    # Convert symbolic inputs to floats for execution rollout
    u_float = np.array([[float(u_opt[i, t]) for t in range(u_opt.shape[1])] for i in range(u_opt.shape[0])])
    
    # Rollout trajectory
    x_roll = np.zeros((3, T + 1), dtype=float)
    x_roll[:, 0] = x0
    for t in range(T):
        x_roll[:, t + 1] = np.array([float(v) for v in sys.f(x_roll[:, t], u_float[:, t])])
        
    return x_roll
