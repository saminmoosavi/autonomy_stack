(define (problem factory_problem)
  (:domain factory_jackal)

  (:objects
    jackal1 - robot

    aisle1 aisle2 - aisle
    shelfA shelfB - shelf_zone
    dock1 - loading_zone

    box1 - box
  )

  (:init
    (at jackal1 aisle1)
    (localized jackal1)
    (battery-ok jackal1)
    (available jackal1)

    (connected aisle1 aisle2)
    (connected aisle2 shelfA)

    (safe aisle1)
    (safe aisle2)
    (safe shelfA)
    (shelf-access shelfA)

    (box-at box1 dock1)

    (free-gripper jackal1)

    (= (battery-level jackal1) 100)
  )

  (:goal
    (and
      (inspected shelfA)
    )
  )
)
