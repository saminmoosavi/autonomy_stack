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
  )

  (:init
    ;; Robot state -- R5 is the hub and the spawn region (SPAWN_X/Y put the
    ;; jackal at (-0.2, 1.0), nearest centroid R5 at (1.44, 2.74)).
    (at jackal_1 R5)
    (free-gripper jackal_1)

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
    )
  )
)
