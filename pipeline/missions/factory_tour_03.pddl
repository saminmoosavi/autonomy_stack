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
;; Objects, types and connectivity are inherited verbatim from
;; factory_tour_01.pddl; every region outside R1/R2/R5 is dropped along with
;; each fact that named one.
;;
;; THE GOAL STATES THE WHOLE MISSION: tour the three regions AND end up at the
;; cone. Both halves matter. Coverage alone was what this file used to ask for,
;; and it left the object the mission is named after out of the problem
;; entirely -- the region to approach was computed in Python after the tour and
;; injected as a bare (at jackal_1 r1) goal, so nothing the planner or the LLM
;; read ever mentioned a cone.
;;
;; THIS PROBLEM IS DELIBERATELY UNSOLVABLE AS WRITTEN. Classical planning
;; cannot sense, and the cone's region is not derivable from anything in this
;; file. (location-unknown cone) says exactly that, and it is `approach`'s
;; negative precondition, so (reached jackal_1 cone) has no achiever. Fast
;; Downward will report the problem unsolvable, and it is right to.
;;
;; The plan is not what the caller keeps -- the caller keeps its executable
;; PREFIX. plan_prefix.executable_prefix forward-simulates the returned actions
;; and stops at the first one whose preconditions are unmet, which is precisely
;; the cone action. What survives is the (visited ?l) conjuncts' work: the
;; survey of every region the cone could be in. Drive that, let perception
;; report what it saw, then replan the approach against a real position.
;;
;; Run it that way with planner_mode "evoplan_only", which skips Fast Downward.
;; The unsolvability pre-check is a sound guard everywhere else -- handing an
;; impossible problem to an LLM burns the deliberation budget for nothing --
;; but here it rejects the exact case the prefix exists for.
;;
;; An earlier revision added a `search-for` action letting the planner ASSUME
;; the discovery in a (may-contain cone ?l) region, which made the goal
;; reachable and left nothing to truncate. Both were removed: stating a goal
;; the planner genuinely cannot meet, and taking what it can, is the honest
;; version of the same idea.
;;
;; When perception localises the cone for real, build_runtime_problem retracts
;; (location-unknown cone), asserts (object-at cone <region>) and narrows the
;; goal to (reached jackal_1 cone) -- solvable, and planned against fact.
;;
;; That retraction is why the explanation lives up here and not next to the
;; fact it describes. remove_init_fact deletes the (location-unknown cone)
;; LINE; an adjacent comment survives it. This problem text is not only a
;; diagnostic artifact -- it is the seed EvoPlan hands the LLM -- so a comment
;; reading "nobody knows where the cone is", left stranded a few lines above a
;; spliced-in (object-at cone r1), is a contradiction planted in the model's
;; input. Comments in (:init ...) must therefore describe only facts that are
;; never retracted.
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

    ;; The object the mission is looking for. The symbol is resolved by TYPE,
    ;; not by name: the service picks the sole declared `target` (or slug-
    ;; matches the detector class when a problem declares several), so this can
    ;; stay readable PDDL while perception calls it "traffic cone".
    cone      - target
  )

  (:init
    ;; Robot state. R5 is the hub and the spawn region: SPAWN_X/SPAWN_Y put the
    ;; jackal at (-0.2, 1.0), whose nearest centroid is R5 at (1.44, 2.74).
    (at jackal_1 R5)
    (free-gripper jackal_1)
    (location-unknown cone)   ;; retracted on discovery -- see the header

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
      (reached jackal_1 cone)
    )
  )
)
