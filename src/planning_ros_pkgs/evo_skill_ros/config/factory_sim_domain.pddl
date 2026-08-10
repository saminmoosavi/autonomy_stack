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
  ;; ------------------------------------------------------------
  (:types
    robot
    location
    aisle shelf_zone loading_zone charging_zone - location
    box
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

    ;; Mission status (asserted by action effects)
    (inspected ?s - shelf_zone)
    (visited ?l - location)
    (delivered ?b - box ?l - location)
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
)
