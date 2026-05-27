import numpy as np
import matplotlib.pyplot as plt
import sys
import types

from stlpy.STL import LinearPredicate
from stlpy.systems import NonlinearSystem
from stlpy.solvers import DrakeSmoothSolver
import pydrake.symbolic as sym





# from pydrake.solvers import (
#     IpoptSolver,
#     SnoptSolver,
#     SolverOptions,
#     CommonSolverOption,
# )

def wrap_angle(theta: float) -> float:
    """Wrap angle to [-pi, pi)."""
    return (theta + np.pi) % (2.0 * np.pi) - np.pi


def make_unicycle_context_system(
    dt: float,
    cone_xy: tuple[float, float],
    goal_xy: tuple[float, float],
    distance_tolerance: float,
    cone_radius: float = 0.15,
    goal_radius: float = 0.25,
) -> NonlinearSystem:
    """
    State:
        x = [px, py, theta]
    Input:
        u = [v, omega]
    Output:
        y = [px, py, theta, cone_clearance, goal_margin]

    cone_clearance >= 0  => safe wrt red cone
    goal_margin    >= 0  => inside goal region
    """
    cx, cy = cone_xy
    gx, gy = goal_xy
    safe_radius = cone_radius + distance_tolerance

    def f(x, u):
        px, py, th = x
        v, om = u
        # print(type(v), type(om))
        # print(type(px), type(py), type(th))

        x_next = np.array([
            px + dt * v * np.cos(th),
            py + dt * v * np.sin(th),
            th+dt*om, #wrap_angle(th + dt * om),
        ])
        return x_next

    def g(x, u):
        px, py, th = x

        # cone_dist = sym.sqrt((px - cx) ** 2 + (py - cy) ** 2)
        # cone_clearance = cone_dist - safe_radius

        # goal_dist = sym.sqrt((px - gx) ** 2 + (py - gy) ** 2)
        # goal_margin = goal_radius - goal_dist

        cone_clearance = (px - cx)**2 + (py - cy)**2 - safe_radius**2
        goal_margin = goal_radius**2 - ((px - gx)**2 + (py - gy)**2)

        y = np.array([
            px,
            py,
            th,
            cone_clearance,
            goal_margin,
        ])
        return y

    return NonlinearSystem(f=f, g=g, n=3, m=2, p=5)


def make_spec(T: int):
    """
    STL formula:
        Always avoid red cone  AND  Eventually reach goal
    over horizon [0, T].
    """
    # y = [px, py, theta, cone_clearance, goal_margin]

    cone_safe = LinearPredicate(
        a=np.array([0.0, 0.0, 0.0, 1.0, 0.0]),
        b=0.0,
    )

    at_goal = LinearPredicate(
        a=np.array([0.0, 0.0, 0.0, 0.0, 1.0]),
        b=0.0,
    )



    phi = cone_safe.always(0, T) & at_goal.eventually(0, T)
    # print ("phi", phi)
    return phi


def rollout_system(sys: NonlinearSystem, x0: np.ndarray, u: np.ndarray):
    """
    Roll out the nonlinear system using the solved controls.

    Parameters
    ----------
    sys : NonlinearSystem
    x0  : (3,) initial state
    u   : (2, T) control sequence

    Returns
    -------
    x : (3, T+1)
    y : (5, T+1)
    """
    T = u.shape[1]
    x = np.zeros((3, T + 1), dtype=float)
    y = np.zeros((5, T + 1), dtype=float)

    x[:, 0] = x0
    y[:, 0] = sys.g(x[:, 0], np.zeros(2))

    for t in range(T):
        # x[:, t + 1] = sys.f(x[:, t], u[:, t])
        print([[v] for v in sys.f(x[:, t], u[:, t])])
        print(type(sys.f(x0, u)[0]))
        x[:, t + 1] = np.array([float(v) for v in sys.f(x[:, t], u[:, t])])
        # Use u[:, t] for output at the new state only if you want control-dependent output.
        # Here output depends only on state, so zeros are fine.
        # y[:, t + 1] = sys.g(x[:, t + 1], np.zeros(2))
        y[:, t + 1] = np.array([float(v) for v in sys.g(x[:, t + 1], np.zeros(2))])


    return x, y


def solve_problem():
    # -----------------------------
    # User-provided problem data
    # -----------------------------
    dt = 0.1
    T = 123

    x0 = np.array([11.860, 17.750, 2.728], dtype=float)

    cone_xy = (7.825, 19.025) #red cone location
    goal_xy = (1.890, 22.130)          # goal location

    distance_tolerance = 0.3    # user-provided safety clearance
    cone_radius = 0.15            # physical cone radius
    goal_radius = 0.30            # how close counts as "reached goal"

    # -----------------------------
    # Build system + STL spec
    # -----------------------------
    sys = make_unicycle_context_system(
        dt=dt,
        cone_xy=cone_xy,
        goal_xy=goal_xy,
        distance_tolerance=distance_tolerance,
        cone_radius=cone_radius,
        goal_radius=goal_radius,
    )
    phi = make_spec(T)

    # DrakeSmoothSolver expects x0 as an (n,1) numpy matrix per docs.
    x0_col = x0.reshape(-1, 1)

    solver = DrakeSmoothSolver(
        spec=phi,
        sys=sys,
        x0=x0_col,
        T=T,
        k=10.0,         # smoothing tightness
        verbose=True,
    )

    # -----------------------------
    # Bounds
    # -----------------------------
    # controls: u = [v, omega]
    u_min = np.array([0.0, -1.5], dtype=float)
    u_max = np.array([1.0,  1.5], dtype=float)
    solver.AddControlBounds(u_min, u_max)

    # states: [px, py, theta]
    x_min = np.array([-100.0, -300.0, -2.0 * np.pi], dtype=float)
    x_max = np.array([ 500.0,  300.0,  2.0 * np.pi], dtype=float)
    solver.AddStateBounds(x_min, x_max)

    # Running cost to discourage wild motion
    Q = 0.01 * np.eye(3)
    R = np.diag([0.05, 0.05])
    solver.AddQuadraticCost(Q, R)

    # Enforce actual satisfaction, not just maximize smooth robustness
    solver.AddRobustnessConstraint(rho_min=0.0)

    # -----------------------------
    # Solve
    # -----------------------------
    x_opt, u_opt, rho_opt, solve_time = solver.Solve()


    # rollout expects float but solver returns symbolic, convert them to array of floats before rollout
    u_opt = np.array(
        [[float(u_opt[i, t]) for t in range(u_opt.shape[1])]
        for i in range(u_opt.shape[0])],
        dtype=float
    )
    x_opt = np.array(
        [[float(x_opt[i, t]) for t in range(x_opt.shape[1])]
        for i in range(x_opt.shape[0])],
        dtype=float
    )

    print(type(u_opt))
    print(type(u_opt.flat[0]))
    # x_opt is documented as (n, T)
    # u_opt is documented as (m, T)
    # Depending on version, x_opt may omit x0. We re-rollout from x0 for consistency.
    x_roll, y_roll = rollout_system(sys, x0, u_opt)
    # send it to nav2

    print(f"Optimal robustness: {rho_opt:.4f}")
    print(f"Solve time: {solve_time:.3f} s")
    print(f"Minimum cone clearance over rollout: {np.min(y_roll[3, :]):.4f}")
    print(f"Maximum goal margin over rollout:    {np.max(y_roll[4, :]):.4f}")



    # -----------------------------
    # -----------------------------
    # Plot
    # -----------------------------
    fig, axes = plt.subplots(1, 2, figsize=(13, 5))

    # Trajectory plot
    ax = axes[0]
    ax.plot(x_roll[0, :], x_roll[1, :], "-o", markersize=3, label="trajectory")
    ax.plot(x0[0], x0[1], "ks", label="start")
    ax.plot(goal_xy[0], goal_xy[1], "g*", markersize=14, label="goal center")
    ax.plot(cone_xy[0], cone_xy[1], "r^", markersize=10, label="red cone")

    safe_radius = cone_radius + distance_tolerance
    cone_circle = plt.Circle(cone_xy, safe_radius, fill=False, linestyle="--")
    goal_circle = plt.Circle(goal_xy, goal_radius, fill=False, linestyle=":")
    ax.add_patch(cone_circle)
    ax.add_patch(goal_circle)

    # draw a few heading arrows
    step = max(1, (x_roll.shape[1] // 12))
    for k in range(0, x_roll.shape[1], step):
        px, py, th = x_roll[:, k]
        ax.arrow(
            px, py,
            0.15 * np.cos(th),
            0.15 * np.sin(th),
            head_width=0.06,
            length_includes_head=True,
        )

    ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_title("Unicycle trajectory with STL safety and goal constraints")
    ax.legend()
    ax.grid(True)

    # Robustness-relevant channels
    ax2 = axes[1]
    tgrid = np.arange(y_roll.shape[1]) * dt
    ax2.plot(tgrid, y_roll[3, :], label="cone clearance")
    ax2.plot(tgrid, y_roll[4, :], label="goal margin")
    ax2.axhline(0.0, linestyle="--")
    ax2.set_xlabel("time [s]")
    ax2.set_ylabel("margin")
    ax2.set_title("Predicate channels")
    ax2.legend()
    ax2.grid(True)

    plt.tight_layout()
    plt.show()

    return {
        "x_opt": x_opt,
        "u_opt": u_opt,
        "rho_opt": rho_opt,
        "solve_time": solve_time,
        "x_roll": x_roll,
        "y_roll": y_roll,
    }


if __name__ == "__main__":


    solve_problem()
