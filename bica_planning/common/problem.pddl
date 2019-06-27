(define (problem task)
(:domain default_domain)
(:objects
    Jack - person
    m1 - message
    leia - robot
    bedroom kitchen - room
)
(:init
    (person_at Jack kitchen)

    (robot_at leia kitchen)

    (robot_talk leia m1 jack)

    (robot_near_person leia jack)

)
(:goal (and
    (robot_talk leia m1 Jack)
))
)
