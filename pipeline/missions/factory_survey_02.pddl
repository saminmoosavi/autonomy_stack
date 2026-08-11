;; Open-world find-and-inspect over R1-R5: search the regions, and inspect every
;; object of the mission's target class that turns up.
;;
;; HOW MANY THERE ARE AND WHERE THEY ARE IS NOT KNOWN, and nothing in this file
;; may imply otherwise. That is not modesty about the encoding, it is the
;; experiment: this text is the seed EvoPlan hands the LLM, so a comment naming
;; the object count, their coordinates, or which regions hold them would let a
;; model write the plan straight out of the prose with perception contributing
;; nothing -- and the run would still look like a success. Ground truth about
;; the world belongs in the world file and in the write-up, never here.
;;
;; WHAT THE MISSION IS. Cover R1-R5 and inspect every object of the classes the
;; operator names in INSPECT_OBJECTS (a traffic cone, for the runs this mission
;; was written for). That intent is a comment rather than a fact because
;; classical PDDL cannot state it: `(:objects ...)` must declare every object
;; the problem names, and the objects here are the unknown. A goal quantified
;; over "every cone" has nothing to range over until perception has found one.
;;
;; So the goal below is the SEARCH, and only the search, which is solvable
;; exactly as written. The inspections are added to it at runtime: the executor
;; clusters detections into individual objects
;; (observation_memory.cluster_detections), mints a stable PDDL name for each
;; (object_registry), and build_runtime_problem splices `<class>_1 - target`
;; into (:objects ...), `(object-at <class>_1 <region>)` into (:init ...) and
;; `(inspected-object <class>_1)` into the goal, once per object discovered.
;; The problem therefore GROWS as the robot learns, and its final form is a
;; result of the run rather than an input to it. See pipeline/README.md.
;;
;; EVERY REGION IS IN THE GOAL, including any that turn out to hold nothing.
;; A survey that visited only the regions where something was expected would not
;; be a survey, and "there is nothing in that region" is a finding this mission
;; is capable of producing.
;;
;; Connectivity is the R1-R5 induced subgraph of graph.json, verbatim: R1-R2,
;; R2-R3, R3-R4, R4-R5, R1-R5, R2-R5, R3-R5. Note R4 hangs off R3 and R5 only.
;; Every region named here is declared here; build_runtime_problem filters
;; runtime facts against these declarations because an undeclared name aborts
;; Fast Downward's translator (exit 31) and kills the whole replan.
(define (problem factory_survey_02)
  (:domain factory_jackal)

  (:objects
    jackal_1     - robot
    R5           - aisle
    R1 R2 R3 R4  - shelf_zone
    ;; No targets. Discovered objects are spliced in here at runtime; see the
    ;; header. An empty target set is not an omission, it is the premise.
  )

  (:init
    ;; R5 is the hub and the spawn region: SPAWN_X/SPAWN_Y put the jackal at
    ;; (-0.2, 1.0), whose nearest centroid is R5 at (1.44, 2.74).
    (at jackal_1 R5)
    (free-gripper jackal_1)

    ;; Connectivity (bidirectional), the R1-R5 induced subgraph of graph.json.
    (connected R1 R2) (connected R2 R1)
    (connected R2 R3) (connected R3 R2)
    (connected R3 R4) (connected R4 R3)
    (connected R4 R5) (connected R5 R4)
    (connected R1 R5) (connected R5 R1)
    (connected R2 R5) (connected R5 R2)
    (connected R3 R5) (connected R5 R3)

    (pickup-zone R1)
  )

  ;; The search, and only the search. Every (inspected-object ...) conjunct is
  ;; appended at runtime as objects are found; goal_conjuncts/set_goal keep
  ;; these five, so a replan issued mid-survey finishes the sweep instead of
  ;; abandoning it for the first object that turned up.
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
