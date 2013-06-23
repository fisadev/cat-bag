# coding: utf-8
import functools
from simpleai.search import SearchProblem, astar
from duinobot import Board, Robot

# you have to calibrate and configure this!
DEVICE = '/dev/ttyUSB0'
FORWARD_PARAMS = 35, 4
TURN_PARAMS = {
    'left': (35, 1.2),
    'right': (35, 1.4),
}

UP = -1, 0
DOWN = 1, 0
LEFT = 0, -1
RIGHT = 0, 1

INITIAL = (0, 0)
INITIAL_DIRECTION = UP

GRID = (
    '   X ',
    'XX   ',
    '   X ',
    '  XX ',
    '     ',
)

GOAL = len(GRID) - 1, len(GRID[0]) - 1

ROTATIONS = {
    (UP, RIGHT): ['turn_right'],
    (UP, DOWN): ['turn_right', 'turn_right'],
    (UP, LEFT): ['turn_left'],
    (RIGHT, DOWN): ['turn_right'],
    (RIGHT, LEFT): ['turn_right', 'turn_right'],
    (RIGHT, UP): ['turn_left'],
    (DOWN, LEFT): ['turn_right'],
    (DOWN, UP): ['turn_right', 'turn_right'],
    (DOWN, RIGHT): ['turn_left'],
    (LEFT, UP): ['turn_right'],
    (LEFT, RIGHT): ['turn_right', 'turn_right'],
    (LEFT, DOWN): ['turn_left'],
}


def get_robot():
    board = Board(DEVICE)
    return Robot(board, 0)


def move_coords(state, action):
    row, col = state
    row_move, col_move = action

    return row + row_move, col + col_move


class PathFinderProblem(SearchProblem):
    def actions(self, state):
        new_coords = [move_coords(state, action)
                      for action in (UP, DOWN, LEFT, RIGHT)]

        new_coords = [(row, col)
                      for row, col in new_coords
                      if 0 <= row < len(GRID) and \
                         0 <= col < len(GRID[0]) and \
                         GRID[row][col] == ' ']

        return new_coords

    def result(self, state, action):
        return action

    def is_goal(self, state):
        return state == GOAL

    def cost(self, state1, action, state2):
        return 1

    def heuristic(self, state):
        row, col = state
        row_goal, col_goal = GOAL
        return abs(row - row_goal) + abs(col - col_goal)


def execute_path(path):
    robot = get_robot()
    move_robot = functools.partial(move, robot)

    last_state = INITIAL
    current_direction = INITIAL_DIRECTION

    for action, state in path:
        if action:
            last_row, last_col = last_state
            row, col = state

            movement = row - last_row, col - last_col

            last_state = state

            if current_direction != movement:
                rotations = ROTATIONS[current_direction, movement]
                map(move_robot, rotations)

            current_direction = movement

            move_robot('forward')


def move(robot, movement):
    if movement == 'forward':
        robot.forward(*FORWARD_PARAMS)
        # TODO this should be configurable (it's a small angle correction)
        robot.turnLeft(35, 0.1)
    elif movement == 'turn_left':
        robot.turnLeft(*TURN_PARAMS['left'])
    elif movement == 'turn_right':
        robot.turnRight(*TURN_PARAMS['right'])


if __name__ == '__main__':
    result = astar(PathFinderProblem(INITIAL))

    text_grid = lambda grid: '\n'.join(''.join(row) for row in grid)

    print '--- Grid ---'
    print text_grid(GRID)

    path_grid = [list(row) for row in GRID]

    for action, state in result.path():
        path_grid[state[0]][state[1]] = '*'

    print '--- Path ---'
    print text_grid(path_grid)

    print 'Moving robot!'
    execute_path(result.path())


