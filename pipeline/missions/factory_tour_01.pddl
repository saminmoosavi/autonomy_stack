;; Region-coverage tour: visit every region so perception can log what is
;; where. Deliberately uses ONLY (visited ?l), which move's effect already
;; asserts -- so every action in a solution is a `move`, the one action the
;; executor can actually drive. No pickup/dropoff/inspect, no surrogate
;; executors required.
;;
;; Objects and connectivity are inherited from factory_mission_08.pddl; only
;; the goal differs.
;;
;; The goal states the whole mission: cover every region AND end up at the
;; cone. The cone is declared but its position is not -- (location-unknown
;; cone) says so, and that fact is `approach`'s negative precondition, so
;; (reached jackal_1 cone) has no achiever and THIS PROBLEM IS DELIBERATELY
;; UNSOLVABLE as written. That is the design: the goal states what the mission
;; wants, the planner cannot fully deliver it, and the caller takes the plan's
;; executable PREFIX -- the region survey -- and drives that. Perception then
;; supplies the cone's real region, the replan retracts (location-unknown
;; cone), asserts (object-at cone <region>) and narrows the goal to
;; (reached jackal_1 cone), which IS solvable.
;; factory_tour_03.pddl carries the full rationale.
;;
;; The explanation lives here rather than beside the fact because
;; remove_init_fact deletes the fact's LINE and leaves any adjacent comment
;; behind -- and this text is the seed EvoPlan gives the LLM, so a stranded
;; "nobody knows where the cone is" above a spliced-in (object-at cone r1)
;; would contradict the state it is meant to explain.
(define (problem factory_tour_01)
  (:domain factory_jackal)

  (:objects
    jackal_1                                    - robot
    R5 R6 R9 R13       - aisle
    R1 R2 R3 R4 R7 R8 R10 R11 R12 - shelf_zone
    R14                                         - loading_zone
    charge_dock                                 - charging_zone
    box_1 box_2                                 - box

    ;; The object the mission is looking for, and named in the goal; see
    ;; factory_tour_03.pddl for how a goal over an unlocated object is solvable.
    cone                                        - target
  )

  (:init
    ;; Robot state
    (at jackal_1 R5)
    (free-gripper jackal_1)
    (location-unknown cone)   ;; retracted on discovery -- see the header

    ;; Connectivity (bidirectional)
    (connected R1 R2) (connected R1 R5) (connected R10 R11) (connected R10 R9)
    (connected R11 R10) (connected R11 R12) (connected R12 R11) (connected R12 R13)
    (connected R13 R12) (connected R13 R14) (connected R13 charge_dock) (connected R14 R13)
    (connected R2 R1) (connected R2 R3) (connected R2 R5) (connected R3 R2) (connected R3 R4)
    (connected R3 R5) (connected R4 R3) (connected R4 R5) (connected R5 R1) (connected R5 R2)
    (connected R5 R3) (connected R5 R4) (connected R5 R6) (connected R5 R7) (connected R6 R5)
    (connected R6 R8) (connected R7 R5) (connected R7 R8) (connected R7 R9) (connected R8 R6)
    (connected R8 R7) (connected R8 R9) (connected R9 R10) (connected R9 R7) (connected R9 R8)
    (connected charge_dock R13)

    ;; Location role flags
    (pickup-zone R1)
    (pickup-zone R14)
    (dropoff-zone R7)
    (dropoff-zone R10)
    (shelf-access R10)
    (shelf-access R12)
    (shelf-access R3)
    (shelf-access R7)

    ;; Box starting positions
    (box-at box_1 R1)
    (box-at box_2 R14)
  )

  (:goal
    (and
      (visited r1)
      (visited r2)
      (visited r3)
      (visited r4)
      (visited r5)
      (visited r6)
      (visited r7)
      (visited r8)
      (visited r9)
      (visited r10)
      (visited r11)
      (visited r12)
      (visited r13)
      (visited r14)
      (reached jackal_1 cone)
    )
  )
)
