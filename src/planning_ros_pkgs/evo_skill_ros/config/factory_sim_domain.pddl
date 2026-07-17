(define (domain factory_jackal)
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
    aisle shelf_zone loading_zone charging_zone - location
    object
    box pallet - object
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

    ;; Objects
    (box-at ?b - box ?l - location)
    (carrying ?r - robot ?b - box)
    (free-gripper ?r - robot)
    (pickup-zone ?l - location)
    (dropoff-zone ?l - location)

    ;; Shelves / destinations
    (shelf-access ?s - shelf_zone)
    (inspected ?s - shelf_zone)

    ;; Mission status
    (visited ?l - location)
    (delivered ?b - box ?l - location)
  )

  (:functions
    (battery-level ?r - robot)
    (move-cost ?from - location ?to - location)
    (risk-level ?l - location)
  )

  ;; ------------------------------------------------------------
  ;; Normal movement through safe, unblocked space
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
  ;; Slow movement through narrow aisles
  ;; Higher duration and higher battery penalty
  ;; ------------------------------------------------------------
  (:durative-action move-through-narrow-aisle
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
  ;; Stop when human is detected in the next location
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
  ;; Pick up a box
  ;; ------------------------------------------------------------
  (:durative-action pickup-box
    :parameters (?r - robot ?b - box ?l - location)
    :duration (= ?duration 3)
    :condition (and
      (at start (available ?r))
      (at start (at ?r ?l))
      (at start (box-at ?b ?l))
      (at start (pickup-zone ?l))
      (at start (free-gripper ?r))
      (over all (not (occupied-by-human ?l)))
    )
    :effect (and
      (at start (not (available ?r)))
      (at end (not (box-at ?b ?l)))
      (at end (carrying ?r ?b))
      (at end (not (free-gripper ?r)))
      (at end (available ?r))
    )
  )

  ;; ------------------------------------------------------------
  ;; Drop off a box
  ;; ------------------------------------------------------------
  (:durative-action dropoff-box
    :parameters (?r - robot ?b - box ?l - location)
    :duration (= ?duration 3)
    :condition (and
      (at start (available ?r))
      (at start (at ?r ?l))
      (at start (carrying ?r ?b))
      (at start (dropoff-zone ?l))
      (over all (not (occupied-by-human ?l)))
    )
    :effect (and
      (at start (not (available ?r)))
      (at end (not (carrying ?r ?b)))
      (at end (box-at ?b ?l))
      (at end (delivered ?b ?l))
      (at end (free-gripper ?r))
      (at end (available ?r))
    )
  )

  ;; ------------------------------------------------------------
  ;; Inspect shelf area
  ;; ------------------------------------------------------------
  (:durative-action inspect-shelf
    :parameters (?r - robot ?s - shelf_zone)
    :duration (= ?duration 6)
    :condition (and
      (at start (available ?r))
      (at start (at ?r ?s))
      (at start (shelf-access ?s))
      (over all (not (occupied-by-human ?s)))
      (over all (safe ?s))
    )
    :effect (and
      (at start (not (available ?r)))
      (at end (inspected ?s))
      (at end (available ?r))
      (at end (decrease (battery-level ?r) 1))
    )
  )

  ;; ------------------------------------------------------------
  ;; Recharge at charging station
  ;; ------------------------------------------------------------
  (:durative-action recharge
    :parameters (?r - robot ?c - charging_zone)
    :duration (= ?duration 20)
    :condition (and
      (at start (available ?r))
      (at start (at ?r ?c))
    )
    :effect (and
      (at start (not (available ?r)))
      (at end (assign (battery-level ?r) 100))
      (at end (battery-ok ?r))
      (at end (available ?r))
    )
  )

  ;; ------------------------------------------------------------
  ;; Mark a location unsafe if risk is high
  ;; This represents perception/safety layer input
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
  ;; Example: box or person moved away
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
    )
  )
)