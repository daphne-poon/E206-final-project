# E206 Motion Planning

# Simulator code
# Daphne Poon and Sabrina Shen


def create_motion_planning_problem():
    current_state = [0, 0, 0, 0]
    desired_state = [20, 5.0, 2.0, 0]
    maxR = 8
    walls = [[-maxR, maxR, maxR, maxR, 2 * maxR], [maxR, maxR, maxR, -maxR, 2 * maxR],
             [maxR, -maxR, -maxR, -maxR, 2 * maxR], [-maxR, -maxR, -maxR, maxR, 2 * maxR]]
    objects = [[4, 0, 1.0], [-2, -3, 1.5]]

    return current_state, desired_state, objects, walls


def main():
    pass


if __name__ == '__main__':
    main()
