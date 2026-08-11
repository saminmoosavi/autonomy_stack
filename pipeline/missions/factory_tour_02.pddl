;; Region-coverage tour restricted to R1-R5: the south block plus the R5 hub.
;;
;; Same shape as factory_tour_01 -- goal is ONLY (visited ?l), which move's
;; effect already asserts, so every action in a solution is a `move`, the one
;; action the executor can actually drive. No pickup/dropoff/inspect, no
;; surrogate executors required.
;;
;; Why a 5-region cut: R1-R5 are a fully-meshed clique around the R5 hub, so
;; the tour is short and every leg is a single graph edge. R1 holds cone_r1
;; (-12.29, -10.69) in warehouse_people0.sdf -- an OpenRobotics Construction
;; Cone, which replaced the backpack that used to sit at that same pose -- and
;; it snaps to R1 at 1.60 m, next-nearest R2 at 7.83 m, so find-object runs
;; have exactly one plausible answer and a wrong one is unambiguous rather
;; than marginal.
;;
;; The goal states the whole mission: cover all five regions AND end up at
;; BOTH searchable objects. Neither position is known -- (location-unknown
;; cone) and (location-unknown skateboard) say so, and each is `approach`'s
;; negative precondition for its own object -- so the two (reached ...)
;; conjuncts have no achiever and THIS PROBLEM IS DELIBERATELY UNSOLVABLE as
;; written. The caller takes the plan's executable PREFIX, the region survey,
;; and drives that; see factory_tour_03.pddl for the full rationale.
;;
;; Two targets rather than one is the point of this mission. The single-object
;; case cannot distinguish "the machinery works" from "the machinery happens to
;; hold the one thing it ever saw": each discovery must add its own
;; (object-at ...) without erasing the other's, and the goal must keep both
;; conjuncts, so the approach ends up planning a route that visits the cone and
;; the skateboard rather than just the nearer one.
;;
;; The world places them apart on purpose. warehouse_people0.sdf has cone_r1 at
;; (-12.29 -10.69), 1.60 m from R1, and skateboard_r3 at (2.75 -11.57), 1.60 m
;; from R3 -- opposite ends of the south aisle, so a plan that reaches one is
;; nowhere near the other and the two-object case cannot pass by accident.
;;
;; The explanation lives here rather than beside the fact because
;; remove_init_fact deletes the fact's LINE and leaves any adjacent comment
;; behind -- and this text is the seed EvoPlan gives the LLM, so a stranded
;; "nobody knows where the cone is" above a spliced-in (object-at cone r1)
;; would contradict the state it is meant to explain.
;;
;; Objects, types and connectivity are inherited verbatim from
;; factory_tour_01.pddl; regions outside R1-R5 are dropped along with every
;; fact that referenced them. Nothing here may name an undeclared object:
;; build_runtime_problem splices runtime (blocked ?r)/(visited ?r) facts in at
;; replan time and filters them against the regions DECLARED here (see
;; regions_in_problem), and VAL rejects a problem outright if an unknown name
;; slips through -- which surfaces as "the planner failed" rather than "a
;; region name did not match".
(define (problem factory_tour_02)
  (:domain factory_jackal)

  (:objects
    jackal_1     - robot
    R5           - aisle
    R1 R2 R3 R4  - shelf_zone

    ;; The objects the mission is looking for, both named in the goal; see
    ;; factory_tour_03.pddl for how a goal over an unlocated object is
    ;; solvable. Resolved by TYPE and then by name, so the PDDL symbols need
    ;; not match the detector classes: "traffic cone" slug-matches its last
    ;; word to `cone`, and "skateboard" matches outright. With two targets
    ;; declared the sole-target fallback no longer applies, so a detector class
    ;; that matches neither resolves to nothing rather than to the wrong one.
    cone skateboard - target
  )

  (:init
    ;; Robot state -- R5 is the hub and the spawn region (SPAWN_X/Y put the
    ;; jackal at (-0.2, 1.0), nearest centroid R5 at (1.44, 2.74)).
    (at jackal_1 R5)
    (free-gripper jackal_1)
    ;; Neither position is known. Each is retracted independently as
    ;; perception finds it, so the problem can name one object's region while
    ;; the other is still being searched for.
    (location-unknown cone)
    (location-unknown skateboard)

    ;; Connectivity (bidirectional), the R1-R5 induced subgraph of tour_01.
    ;; R1-R3 and R1-R4 are absent in tour_01 too: R1's only neighbours are R2
    ;; and R5, so a tour cannot shortcut across the block.
    (connected R1 R2) (connected R1 R5)
    (connected R2 R1) (connected R2 R3) (connected R2 R5)
    (connected R3 R2) (connected R3 R4) (connected R3 R5)
    (connected R4 R3) (connected R4 R5)
    (connected R5 R1) (connected R5 R2) (connected R5 R3) (connected R5 R4)

    ;; Location role flags. Only those naming R1-R5 survive; the box facts from
    ;; tour_01 are dropped entirely because box_2 lived at R14 and no goal here
    ;; touches a box.
    (pickup-zone R1)
    (shelf-access R3)
  )

  (:goal
    (and
      (visited r1)
      (visited r2)
      (visited r3)
      (visited r4)
      (visited r5)
      (reached jackal_1 cone)
      (reached jackal_1 skateboard)
    )
  )
)
