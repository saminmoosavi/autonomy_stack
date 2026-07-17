(define (domain lens_lab)
  (:requirements
    :strips
    :typing
    :negative-preconditions
    :durative-actions
    :fluents
    :conditional-effects
  )

  (:types
    robot
    location
    lab_region workstation storage_zone safety_zone entrance_zone - location
    object
    lab_equipment furniture waste_bin display surface - object
    human
  )

  (:predicates
    ;; Robot state
    (at ?r - robot ?l - location)
    (connected ?from - location ?to - location)
    (localized ?r - robot)
    (battery-ok ?r - robot)
    (available ?r - robot)

    ;; Navigation/environment state
    (blocked ?l - location)
    (occupied-by-human ?l - location)
    (narrow ?l - location)
    (safe ?l - location)
    (emergency-area ?l - location)

    ;; Lab objects from graph_lab.json
    (object-at ?o - object ?l - location)
    (equipment ?o - lab_equipment)
    (furniture-item ?o - furniture)
    (waste-container ?o - waste_bin)
    (display-device ?o - display)
    (work-surface ?o - surface)
    (inspected-object ?o - object)
    (clean ?o - object)

    ;; Lab zones / capabilities
    (inspection-zone ?l - location)
    (compute-zone ?l - location)
    (fabrication-zone ?l - location)
    (robot-arm-zone ?l - location)
    (seating-zone ?l - location)
    (fire-safety-zone ?l - location)
    (clear-access ?l - location)

    ;; Mission status
    (visited ?l - location)
    (checked-zone ?l - location)
    (reported ?o - object)
  )

  (:functions
    (battery-level ?r - robot)
    (move-cost ?from - location ?to - location)
    (risk-level ?l - location)
  )

  ;; ------------------------------------------------------------
  ;; Normal movement through safe, unblocked lab regions
  ;; ------------------------------------------------------------
  (:durative-action move
    :parameters (?r - robot ?from - location ?to - location)
    :duration (= ?duration (move-cost ?from ?to))
    :condition (and
      (at start (available ?r))
      (at start (localized ?r))
      (at start (battery-ok ?r))
      (at start (at ?r ?from))
      (at start (connected ?from ?to))
      (over all (not (blocked ?to)))
      (over all (not (occupied-by-human ?to)))
      (over all (safe ?to))
    )
    :effect (and
      (at start (not (available ?r)))
      (at end (not (at ?r ?from)))
      (at end (at ?r ?to))
      (at end (visited ?to))
      (at end (available ?r))
      (at end (decrease (battery-level ?r) 2))
    )
  )

  ;; ------------------------------------------------------------
  ;; Slower movement through cluttered seating, desk, or equipment areas
  ;; ------------------------------------------------------------
  (:durative-action move-through-narrow-area
    :parameters (?r - robot ?from - location ?to - location)
    :duration (= ?duration (* 2 (move-cost ?from ?to)))
    :condition (and
      (at start (available ?r))
      (at start (localized ?r))
      (at start (battery-ok ?r))
      (at start (at ?r ?from))
      (at start (connected ?from ?to))
      (at start (narrow ?to))
      (over all (not (blocked ?to)))
      (over all (not (occupied-by-human ?to)))
      (over all (safe ?to))
    )
    :effect (and
      (at start (not (available ?r)))
      (at end (not (at ?r ?from)))
      (at end (at ?r ?to))
      (at end (visited ?to))
      (at end (available ?r))
      (at end (decrease (battery-level ?r) 4))
    )
  )

  ;; ------------------------------------------------------------
  ;; Wait when a person is occupying the current lab region
  ;; ------------------------------------------------------------
  (:durative-action wait-for-human
    :parameters (?r - robot ?l - location)
    :duration (= ?duration 5)
    :condition (and
      (at start (at ?r ?l))
      (at start (available ?r))
    )
    :effect (and
      (at end (available ?r))
    )
  )

  ;; ------------------------------------------------------------
  ;; Inspect lab equipment such as the server rack, 3D printer,
  ;; lambda machines, carts, or robot arm table.
  ;; ------------------------------------------------------------
  (:durative-action inspect-equipment
    :parameters (?r - robot ?o - lab_equipment ?l - location)
    :duration (= ?duration 6)
    :condition (and
      (at start (available ?r))
      (at start (at ?r ?l))
      (at start (object-at ?o ?l))
      (at start (equipment ?o))
      (at start (inspection-zone ?l))
      (over all (not (occupied-by-human ?l)))
      (over all (safe ?l))
    )
    :effect (and
      (at start (not (available ?r)))
      (at end (inspected-object ?o))
      (at end (reported ?o))
      (at end (checked-zone ?l))
      (at end (available ?r))
      (at end (decrease (battery-level ?r) 1))
    )
  )

  ;; ------------------------------------------------------------
  ;; Check desks, table, whiteboard, TV, chairs, bean bag, and cabinet
  ;; for occupancy or blocked access.
  ;; ------------------------------------------------------------
  (:durative-action check-lab-object
    :parameters (?r - robot ?o - object ?l - location)
    :duration (= ?duration 4)
    :condition (and
      (at start (available ?r))
      (at start (at ?r ?l))
      (at start (object-at ?o ?l))
      (at start (clear-access ?l))
      (over all (not (occupied-by-human ?l)))
      (over all (safe ?l))
    )
    :effect (and
      (at start (not (available ?r)))
      (at end (inspected-object ?o))
      (at end (checked-zone ?l))
      (at end (available ?r))
      (at end (decrease (battery-level ?r) 1))
    )
  )

  ;; ------------------------------------------------------------
  ;; Perform a fabrication-room task near the 3D printer or lambda area
  ;; ------------------------------------------------------------
  (:durative-action service-fabrication-zone
    :parameters (?r - robot ?l - location)
    :duration (= ?duration 8)
    :condition (and
      (at start (available ?r))
      (at start (at ?r ?l))
      (at start (fabrication-zone ?l))
      (over all (not (occupied-by-human ?l)))
      (over all (safe ?l))
    )
    :effect (and
      (at start (not (available ?r)))
      (at end (checked-zone ?l))
      (at end (available ?r))
      (at end (decrease (battery-level ?r) 2))
    )
  )

  ;; ------------------------------------------------------------
  ;; Inspect the fire/safety region without entering unsafe space
  ;; ------------------------------------------------------------
  (:durative-action inspect-fire-safety-zone
    :parameters (?r - robot ?l - location)
    :duration (= ?duration 5)
    :condition (and
      (at start (available ?r))
      (at start (at ?r ?l))
      (at start (fire-safety-zone ?l))
      (over all (not (occupied-by-human ?l)))
      (over all (safe ?l))
    )
    :effect (and
      (at start (not (available ?r)))
      (at end (checked-zone ?l))
      (at end (available ?r))
      (at end (decrease (battery-level ?r) 1))
    )
  )

  ;; ------------------------------------------------------------
  ;; Recharge or recover at an accessible workstation/support area
  ;; ------------------------------------------------------------
  (:durative-action recharge
    :parameters (?r - robot ?l - location)
    :duration (= ?duration 20)
    :condition (and
      (at start (available ?r))
      (at start (at ?r ?l))
      (at start (compute-zone ?l))
    )
    :effect (and
      (at start (not (available ?r)))
      (at end (assign (battery-level ?r) 100))
      (at end (battery-ok ?r))
      (at end (available ?r))
    )
  )

  ;; ------------------------------------------------------------
  ;; Mark a region unsafe if the perception/safety layer reports risk
  ;; ------------------------------------------------------------
  (:action mark-unsafe
    :parameters (?l - location)
    :precondition (and
      (not (safe ?l))
    )
    :effect (and
      (blocked ?l)
    )
  )

  ;; ------------------------------------------------------------
  ;; Clear temporary obstacle after perception update
  ;; ------------------------------------------------------------
  (:action clear-location
    :parameters (?l - location)
    :precondition (and
      (blocked ?l)
      (not (occupied-by-human ?l))
    )
    :effect (and
      (not (blocked ?l))
      (safe ?l)
      (clear-access ?l)
    )
  )
)
