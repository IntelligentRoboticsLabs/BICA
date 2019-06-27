(:durative-action talk
    :parameters (?r - robot ?p - person ?m - message)
    :duration ( = ?duration 5)
    :condition (and
        (over all(robot_near_person ?r ?p))
    )
    :effect (and
        (at end(robot_talk ?r ?m ?p))
    )
)

(:durative-action approach
    :parameters (?r - robot ?ro - room ?p - person)
    :duration ( = ?duration 5)
    :condition (and
        (over all(robot_at ?r ?ro))
        (over all(person_at ?p ?ro))
    )
    :effect (and
        (at end(robot_near_person ?r ?p))
    )
)
