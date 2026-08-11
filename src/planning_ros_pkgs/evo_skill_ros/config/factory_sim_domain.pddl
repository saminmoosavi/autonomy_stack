;; ============================================================================
;; Factory domain -- CLASSICAL (STRIPS). Single source of truth for both halves
;; of the pipeline.
;;
;; PROVENANCE: this is a verbatim copy of
;;   evolve_stl_pddl/jackal/in/factory_jackal_domain.pddl
;; which is the canonical original (it lives in the EvoPlan submodule and is
;; what the replan service, Fast Downward and VAL all plan/validate against).
;; The copy exists because this path is referenced from eight places -- run_sim.sh,
;; ran.sh, density_sweep.py, the launch default, and three node parameter
;; defaults -- and because it must be installed into the ROS package share.
;;
;; Keep the two byte-identical below this header. test_domain_consistency.py
;; fails if they drift.
;;
;; WHY CLASSICAL, not the durative model this file used to hold:
;;   * Fast Downward rejects :durative-actions outright (translate exit code 31),
;;     and FD/VAL is what scores every candidate plan.
;;   * The durative `move` required (available/localized/battery-ok/safe), none
;;     of which any factory_mission_*.pddl asserts -- so validating against it
;;     rejected every plan the planner could legally produce.
;;   * Nothing executes durations, battery or narrow-aisle semantics anyway.
;; The durative version is preserved as factory_sim_domain_durative.pddl.archive.
;; ============================================================================

(define (domain factory_jackal)
  (:requirements
    :strips
    :typing
    :negative-preconditions
  )

  ;; ------------------------------------------------------------
  ;; Type hierarchy
  ;;   robot     — the Jackal
  ;;   location  — discrete navigation regions
  ;;     aisle          — open traversal region
  ;;     shelf_zone     — region adjacent to a shelf, inspectable
  ;;     loading_zone   — region where boxes can be picked up
  ;;     charging_zone  — region with a charging dock
  ;;   box       — pickupable payload
  ;;   target    — an object the mission must FIND, then drive to
  ;; ------------------------------------------------------------
  (:types
    robot
    location
    aisle shelf_zone loading_zone charging_zone - location
    box
    target
  )

  (:predicates
    ;; Robot state
    (at ?r - robot ?l - location)
    (connected ?from - location ?to - location)
    (carrying ?r - robot ?b - box)
    (free-gripper ?r - robot)

    ;; Location flags (asserted in :init or by runtime failure facts)
    (blocked ?l - location)
    (pickup-zone ?l - location)
    (dropoff-zone ?l - location)
    (shelf-access ?s - shelf_zone)

    ;; Object state
    (box-at ?b - box ?l - location)

    ;; Find-an-object state.
    ;;
    ;; Classical planning cannot sense, so "where is the cone?" is not
    ;; something a plan can discover -- it is something the RUNTIME discovers
    ;; and the next problem states. (location-unknown ?t) is what a mission
    ;; asserts before the tour, and it is `approach`'s negative precondition,
    ;; so a phase-1 goal of (reached ?r ?t) has NO achiever and the problem is
    ;; deliberately unsolvable.
    ;;
    ;; That is the intended shape, not an oversight. The mission goal states
    ;; what the mission wants; the planner cannot fully deliver it; the caller
    ;; takes the plan's executable PREFIX (plan_prefix.executable_prefix),
    ;; which is the region survey -- drive that, see what perception found,
    ;; then replan the approach against a real position. An earlier revision
    ;; instead added a `search-for` action letting the planner ASSUME the
    ;; discovery, which made the goal reachable and left nothing to truncate;
    ;; it was removed in favour of this.
    ;;
    ;; When perception localises the object, build_runtime_problem retracts
    ;; the flag and asserts (object-at ?t ?l), and only then is the approach
    ;; plannable. The two facts are complementary by construction; nothing here
    ;; enforces it, so nothing should ever assert both.
    (location-unknown ?t - target)
    (object-at ?t - target ?l - location)

    ;; Mission status (asserted by action effects)
    (inspected ?s - shelf_zone)
    (visited ?l - location)
    (delivered ?b - box ?l - location)
    (reached ?r - robot ?t - target)
    ;; One object has been looked at from close range. Distinct from
    ;; (inspected ?s - shelf_zone), which is about a REGION and has no executor:
    ;; this one is about a target the runtime discovered, and `inspect-object`
    ;; below does lower to robot motion.
    (inspected-object ?t - target)
  )

  ;; ------------------------------------------------------------
  ;; Move between connected regions.
  ;; Continuous timing, clearance, and battery constraints live in
  ;; the STL contract attached to this action at runtime.
  ;; ------------------------------------------------------------
  (:action move
    :parameters (?r - robot ?from - location ?to - location)
    :precondition (and
      (at ?r ?from)
      (connected ?from ?to)
      (not (blocked ?to))
    )
    :effect (and
      (not (at ?r ?from))
      (at ?r ?to)
      (visited ?to)
    )
  )

  ;; ------------------------------------------------------------
  ;; Pick up a box at a designated pickup zone.
  ;; ------------------------------------------------------------
  (:action pickup-box
    :parameters (?r - robot ?b - box ?l - location)
    :precondition (and
      (at ?r ?l)
      (box-at ?b ?l)
      (pickup-zone ?l)
      (free-gripper ?r)
    )
    :effect (and
      (not (box-at ?b ?l))
      (carrying ?r ?b)
      (not (free-gripper ?r))
    )
  )

  ;; ------------------------------------------------------------
  ;; Drop off a carried box at a designated dropoff zone.
  ;; ------------------------------------------------------------
  (:action dropoff-box
    :parameters (?r - robot ?b - box ?l - location)
    :precondition (and
      (at ?r ?l)
      (carrying ?r ?b)
      (dropoff-zone ?l)
    )
    :effect (and
      (not (carrying ?r ?b))
      (box-at ?b ?l)
      (delivered ?b ?l)
      (free-gripper ?r)
    )
  )

  ;; ------------------------------------------------------------
  ;; Inspect a shelf at the robot's current shelf_zone.
  ;; Coverage and viewing-pose constraints live in the STL contract.
  ;; ------------------------------------------------------------
  (:action inspect-shelf
    :parameters (?r - robot ?s - shelf_zone)
    :precondition (and
      (at ?r ?s)
      (shelf-access ?s)
    )
    :effect (inspected ?s)
  )

  ;; ------------------------------------------------------------
  ;; Recharge at a charging dock.
  ;; The actual battery dynamics are enforced by the STL contract
  ;; (battery signal must reach b_full within t_charge_max).
  ;; ------------------------------------------------------------
  (:action recharge
    :parameters (?r - robot ?c - charging_zone)
    :precondition (at ?r ?c)
    :effect (visited ?c)
  )

  ;; ------------------------------------------------------------
  ;; Stand at the object's region and declare it reached.
  ;;
  ;; Zero-cost in practice: the driving is done by the `move` chain that gets
  ;; the robot to ?l, and the executor drops this action (only `move` lowers to
  ;; a Nav2 goal). It exists so the approach goal can be stated as WHAT the
  ;; mission wants -- (reached ?r ?t) -- instead of the region name the caller
  ;; happened to compute. The planner then derives the region from
  ;; (object-at ?t ?l), so the target lives in the problem's facts rather than
  ;; in a `target_region` string the PDDL never sees.
  ;; ------------------------------------------------------------
  (:action approach
    :parameters (?r - robot ?t - target ?l - location)
    :precondition (and
      (at ?r ?l)
      (object-at ?t ?l)
      (not (location-unknown ?t))
    )
    :effect (reached ?r ?t)
  )

  ;; ------------------------------------------------------------
  ;; Inspect a discovered object from its own region.
  ;;
  ;; UNLIKE `approach`, this one HAS an executor. plan_to_nav2_goals lowers it
  ;; to a Nav2 goal pose at ?l oriented at the object's map position, followed
  ;; by a stationary dwell (inspect_dwell_s, 5 s) while the robot faces it. So
  ;; an inspect-object in a plan costs real mission time and real motion, and
  ;; the region it names is where the robot ends up.
  ;;
  ;; Preconditions are `approach`'s, deliberately: an object can only be
  ;; inspected once the runtime has localised it, which is what retracts
  ;; (location-unknown ?t) and asserts (object-at ?t ?l). The difference is the
  ;; effect -- (reached ?r ?t) says "I got there", (inspected-object ?t) says
  ;; "I got there AND held a view of it" -- and only the latter is what a
  ;; find-and-inspect mission asks for.
  ;;
  ;; ?t is NOT declared by the mission problem in that style of mission. The
  ;; quantity of objects is unknown until perception has run, so
  ;; build_runtime_problem splices both the (:objects ...) declarations and the
  ;; goal conjuncts as instances are discovered. See pipeline/missions/
  ;; factory_survey_01.pddl.
  ;; ------------------------------------------------------------
  (:action inspect-object
    :parameters (?r - robot ?t - target ?l - location)
    :precondition (and
      (at ?r ?l)
      (object-at ?t ?l)
      (not (location-unknown ?t))
    )
    :effect (inspected-object ?t)
  )
)
