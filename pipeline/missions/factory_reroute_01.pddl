;; factory_reroute_01 -- a mission a symbolic replan can actually reroute.
;;
;; The survey missions require every region they drive through, so asserting
;; (blocked ?l) for any of them disconnects the goal and the replan service has
;; to relax the block: the "replanned" plan is the one that just failed. Here
;; only R1 and R3 are required. R2 and R4 are merely ways of getting to R3, one
;; from the west and one from the east:
;;
;;   nominal          R5 -> R1 -> R2 -> R3 -> R2 -> R5        (cost 5)
;;   with R2 blocked  R5 -> R1 -> R5 -> R4 -> R3 -> R4 -> R5  (cost 6)
;;
;; so a person holding up R2 is something the planner can route around instead
;; of something it can only wait out. The detour is strictly longer, which is
;; the point: the planner takes it because of the block and for no other
;; reason. graph.json also connects R3 to R5 directly; that edge is left out
;; here because with it the two routes tie and Fast Downward's tie-break, not
;; the block, would decide which one a replan returns.
(define (problem factory_reroute_01)
  (:domain factory_jackal)

  (:objects
    jackal_1     - robot
    R5           - aisle
    R1 R2 R3 R4  - shelf_zone
  )

  (:init
    (at jackal_1 R5)
    (free-gripper jackal_1)

    ;; Connectivity (bidirectional): the R1-R5 subgraph of graph.json minus R3-R5.
    (connected R1 R2) (connected R2 R1)
    (connected R2 R3) (connected R3 R2)
    (connected R3 R4) (connected R4 R3)
    (connected R4 R5) (connected R5 R4)
    (connected R1 R5) (connected R5 R1)
    (connected R2 R5) (connected R5 R2)

    (pickup-zone R1)
  )

  (:goal
    (and
      (visited r1)
      (visited r3)
      (at jackal_1 r5)
    )
  )
)
