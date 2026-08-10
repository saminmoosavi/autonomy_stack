;; Region-coverage tour: visit every region so perception can log what is
;; where. Deliberately uses ONLY (visited ?l), which move's effect already
;; asserts -- so every action in a solution is a `move`, the one action the
;; executor can actually drive. No pickup/dropoff/inspect, no surrogate
;; executors required.
;;
;; Objects and connectivity are inherited from factory_mission_08.pddl; only
;; the goal differs.
(define (problem factory_tour_01)
  (:domain factory_jackal)

  (:objects
    jackal_1                                    - robot
    R5 R6 R9 R13       - aisle
    R1 R2 R3 R4 R7 R8 R10 R11 R12 - shelf_zone
    R14                                         - loading_zone
    charge_dock                                 - charging_zone
    box_1 box_2                                 - box
  )

  (:init
    ;; Robot state
    (at jackal_1 R5)
    (free-gripper jackal_1)

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
    )
  )
)
