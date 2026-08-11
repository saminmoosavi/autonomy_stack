;; Open-world find-and-inspect: how many objects are out there, and where, is
;; not stated ANYWHERE in this file -- because that is the mission.
;;
;; HOW THIS DIFFERS FROM factory_tour_02/03. Those are closed-world find-an-
;; object missions: they declare `cone - target`, assert (location-unknown cone)
;; and ask for (reached jackal_1 cone). Exactly one object, known to exist,
;; position unknown. The problem is unsolvable as written and the caller drives
;; its executable prefix.
;;
;; Here even the OBJECT SET is unknown. There may be no cones, or six. Nothing
;; below mentions a target, and nothing can: a classical problem must declare
;; every object it names, so declaring the unknown quantity up front would mean
;; inventing it. Instead:
;;
;;   1. this file states the SEARCH -- visit every region -- and nothing else,
;;      so it is perfectly solvable and its plan is a full survey. No prefix
;;      truncation is involved, unlike the tour missions;
;;   2. while that survey is driven, the executor clusters YOLO detections into
;;      individual object instances (observation_memory.cluster_detections) and
;;      mints a stable PDDL name for each (object_registry.ObjectRegistry);
;;   3. every discovery grows the problem: build_runtime_problem splices
;;      `traffic_cone_1 - target` into (:objects ...), (object-at
;;      traffic_cone_1 r3) into (:init ...), and (inspected-object
;;      traffic_cone_1) into the goal below, which is why the goal here is the
;;      floor of what the mission asks rather than the whole of it;
;;   4. the executor replans and drives the inspections. `inspect-object` is
;;      the one non-move action with a real executor: it lowers to a Nav2 goal
;;      at the object's region, oriented to face the object's map position,
;;      followed by a five-second stationary dwell.
;;
;; The mission is complete when the survey is done AND every object discovered
;; has been inspected AND a further poll of perception discovers nothing new.
;; That last clause is the honest reading of "unknown quantity": there is no
;; certificate that all objects have been found, only the absence of new ones.
;;
;; WHICH CLASSES COUNT is set by the executor's inspect_object_classes
;; parameter (INSPECT_OBJECTS in run_trial.sh), not here. This file is
;; class-agnostic on purpose: the same survey serves "inspect every cone" and
;; "inspect every chair", and the regions are the only thing it commits to.
;;
;; Regions and connectivity are the R1/R2/R5 induced subgraph used by
;; factory_tour_03, so a trial of this mission costs about the same drive as
;; that one and the two are directly comparable.
;;
;; Nothing here may name an undeclared object -- build_runtime_problem filters
;; runtime (blocked ?r)/(visited ?r)/(at ?r ?l) facts against the regions
;; DECLARED here, because an undeclared name aborts Fast Downward's translator
;; (exit 31) and kills the whole replan.
(define (problem factory_survey_01)
  (:domain factory_jackal)

  (:objects
    jackal_1  - robot
    R5        - aisle
    R1 R2     - shelf_zone
    ;; No targets. Discovered objects are spliced in here at runtime; see the
    ;; header. An empty target set is not an omission, it is the premise.
  )

  (:init
    ;; R5 is the hub and the spawn region: SPAWN_X/SPAWN_Y put the jackal at
    ;; (-0.2, 1.0), whose nearest centroid is R5 at (1.44, 2.74).
    (at jackal_1 R5)
    (free-gripper jackal_1)

    ;; Connectivity (bidirectional).
    (connected R1 R2) (connected R1 R5)
    (connected R2 R1) (connected R2 R5)
    (connected R5 R1) (connected R5 R2)

    (pickup-zone R1)
  )

  ;; The search, and only the search. Every (inspected-object ...) conjunct is
  ;; appended at runtime as objects are found -- goal_conjuncts/set_goal keep
  ;; these three, so a replan issued mid-survey still finishes the sweep
  ;; instead of abandoning it for the first cone that turned up.
  (:goal
    (and
      (visited r1)
      (visited r2)
      (visited r5)
    )
  )
)
