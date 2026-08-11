# Fixes Needed
## Previous problem.pddl files
```
haozewang@ecen-000429919:/planning/autonomy_stack/results/single_trials$ cat factory_tour_03_p0_evoplan_20260810_184017_replan_1.pddl
;; Minimal find-an-object tour: R5 -> R1 -> R2 -> R5. Three moves.
;;
;; Exists to make the find-object path cheap to test. tour_02's five-region
;; loop spends ~7 minutes driving before the interesting part -- plan
;; exhaustion, the object lookup, and the approach replan -- even begins. This
;; keeps the only region that matters (R1, holding cone_r1) and one more (R2)
;; so the tour is still a tour rather than an out-and-back, then returns to R5
;; so the approach has somewhere to travel FROM.
;;
;; R2 is not padding: with a bare R5 -> R1 -> R5 plan the robot would end at R5
;; having already visited R1, and the approach would be the same single move
;; the tour just made -- a weaker test of retargeting than one where the tour
;; genuinely ends somewhere else.
;;
;; Same shape as factory_tour_01/02: the goal is ONLY (visited ?l), which
;; move's effect already asserts, so every action in a solution is a `move` --
;; the one action the executor can drive. Objects, types and connectivity are
;; inherited verbatim from factory_tour_01.pddl; every region outside R1/R2/R5
;; is dropped along with each fact that named one.
;;
;; Nothing here may name an undeclared object. build_runtime_problem splices
;; runtime (blocked ?r)/(visited ?r)/(at ?r ?l) facts at replan time and filters
;; them against the regions DECLARED here -- a filter that exists because an
;; undeclared (at jackal_1 r6), produced by a drifted odom-frame pose, made
;; Fast Downward's translator abort (exit 31) and killed a whole replan.
(define (problem factory_tour_03)
  (:domain factory_jackal)

  (:objects
    jackal_1  - robot
    R5        - aisle
    R1 R2     - shelf_zone
INSERT EDIT HERE: 
    cone      - position unknown
  )

  (:init
    ;; Robot state. R5 is the hub and the spawn region: SPAWN_X/SPAWN_Y put the
    ;; jackal at (-0.2, 1.0), whose nearest centroid is R5 at (1.44, 2.74).
    (at jackal_1 r1)
    (free-gripper jackal_1)

    ;; Connectivity (bidirectional), the R1/R2/R5 induced subgraph of tour_01.
    (connected R1 R2) (connected R1 R5)
    (connected R2 R1) (connected R2 R5)
    (connected R5 R1) (connected R5 R2)

    ;; Only the role flag naming a surviving region. tour_01's box facts are
    ;; dropped: box_2 lived at R14, and no goal here touches a box.
    (pickup-zone R1)
  )

  (:goal
    (and
      (visited r1)
      (visited r2)
      (visited r5)
    )
  )
)
haozewang@ecen-000429919:/planning/autonomy_stack/results/single_trials$ cat factory_tour_03_p0_evoplan_20260810_184017_replan_2.pddl
;; Minimal find-an-object tour: R5 -> R1 -> R2 -> R5. Three moves.
;;
;; Exists to make the find-object path cheap to test. tour_02's five-region
;; loop spends ~7 minutes driving before the interesting part -- plan
;; exhaustion, the object lookup, and the approach replan -- even begins. This
;; keeps the only region that matters (R1, holding cone_r1) and one more (R2)
;; so the tour is still a tour rather than an out-and-back, then returns to R5
;; so the approach has somewhere to travel FROM.
;;
;; R2 is not padding: with a bare R5 -> R1 -> R5 plan the robot would end at R5
;; having already visited R1, and the approach would be the same single move
;; the tour just made -- a weaker test of retargeting than one where the tour
;; genuinely ends somewhere else.
;;
;; Same shape as factory_tour_01/02: the goal is ONLY (visited ?l), which
;; move's effect already asserts, so every action in a solution is a `move` --
;; the one action the executor can drive. Objects, types and connectivity are
;; inherited verbatim from factory_tour_01.pddl; every region outside R1/R2/R5
;; is dropped along with each fact that named one.
;;
;; Nothing here may name an undeclared object. build_runtime_problem splices
;; runtime (blocked ?r)/(visited ?r)/(at ?r ?l) facts at replan time and filters
;; them against the regions DECLARED here -- a filter that exists because an
;; undeclared (at jackal_1 r6), produced by a drifted odom-frame pose, made
;; Fast Downward's translator abort (exit 31) and killed a whole replan.
(define (problem factory_tour_03)
  (:domain factory_jackal)

  (:objects
    jackal_1  - robot
    R5        - aisle
    R1 R2     - shelf_zone
INSERT EDIT HERE
    cone      - position known
  )

  (:init
    ;; Robot state. R5 is the hub and the spawn region: SPAWN_X/SPAWN_Y put the
    ;; jackal at (-0.2, 1.0), whose nearest centroid is R5 at (1.44, 2.74).
    (at jackal_1 r1)
    (free-gripper jackal_1)

    ;; Connectivity (bidirectional), the R1/R2/R5 induced subgraph of tour_01.
    (connected R1 R2) (connected R1 R5)
    (connected R2 R1) (connected R2 R5)
    (connected R5 R1) (connected R5 R2)

    ;; Only the role flag naming a surviving region. tour_01's box facts are
    ;; dropped: box_2 lived at R14, and no goal here touches a box.
    (pickup-zone R1)
  )

  (:goal
    (and
      (visited r1)
      (visited r2)
      (visited r5)
    )
  )
)
haozewang@ecen-000429919:/planning/autonomy_stack/results/single_trials$ cat factory_tour_03_p0_evoplan_20260810_184017_replan_3.pddl
;; Minimal find-an-object tour: R5 -> R1 -> R2 -> R5. Three moves.
;;
;; Exists to make the find-object path cheap to test. tour_02's five-region
;; loop spends ~7 minutes driving before the interesting part -- plan
;; exhaustion, the object lookup, and the approach replan -- even begins. This
;; keeps the only region that matters (R1, holding cone_r1) and one more (R2)
;; so the tour is still a tour rather than an out-and-back, then returns to R5
;; so the approach has somewhere to travel FROM.
;;
;; R2 is not padding: with a bare R5 -> R1 -> R5 plan the robot would end at R5
;; having already visited R1, and the approach would be the same single move
;; the tour just made -- a weaker test of retargeting than one where the tour
;; genuinely ends somewhere else.
;;
;; Same shape as factory_tour_01/02: the goal is ONLY (visited ?l), which
;; move's effect already asserts, so every action in a solution is a `move` --
;; the one action the executor can drive. Objects, types and connectivity are
;; inherited verbatim from factory_tour_01.pddl; every region outside R1/R2/R5
;; is dropped along with each fact that named one.
;;
;; Nothing here may name an undeclared object. build_runtime_problem splices
;; runtime (blocked ?r)/(visited ?r)/(at ?r ?l) facts at replan time and filters
;; them against the regions DECLARED here -- a filter that exists because an
;; undeclared (at jackal_1 r6), produced by a drifted odom-frame pose, made
;; Fast Downward's translator abort (exit 31) and killed a whole replan.
(define (problem factory_tour_03)
  (:domain factory_jackal)

  (:objects
    jackal_1  - robot
    R5        - aisle
    R1 R2     - shelf_zone
  )

  (:init
    ;; Robot state. R5 is the hub and the spawn region: SPAWN_X/SPAWN_Y put the
    ;; jackal at (-0.2, 1.0), whose nearest centroid is R5 at (1.44, 2.74).
    (at jackal_1 r5)
    (free-gripper jackal_1)

    ;; Connectivity (bidirectional), the R1/R2/R5 induced subgraph of tour_01.
    (connected R1 R2) (connected R1 R5)
    (connected R2 R1) (connected R2 R5)
    (connected R5 R1) (connected R5 R2)

    ;; Only the role flag naming a surviving region. tour_01's box facts are
    ;; dropped: box_2 lived at R14, and no goal here touches a box.
    (pickup-zone R1)
    (visited r1)
    (visited r2)
  )

  (:goal
    (at jackal_1 r1)
  )
)
```
