import numpy as np

class Direction(object):
    def __init__(self, value, char):
        self.value = value
        self.char = char
    def __eq__(self, other):
        if self.char == other.char: return True

class State(object):
    def __init__(self, value=None, parent=None, parent_move=None):
        self.value = value
        self.parent = parent
        self.parent_move = parent_move
        self.f = 0
        self.h = 0
        self.g = 0
    def print(self):
        print("----------")
        print(self.value[0,0], self.value[1,0], self.value[2,0])
        print(self.value[0,1], self.value[1,1], self.value[2,1])
        print(self.value[0,2], self.value[1,2], self.value[2,2])
        print("----------")
    def attr_print(self):
        print("----------")
        print(self.value)
        # print(self.parent)
        print(self.parent_move.char)
        print(self.f)
        print(self.g)
        print(self.h)
    def __eq__(self, other):
        return (self.value == other.value).all()

UP    = Direction(np.array([0,-1]), 'u')
DOWN  = Direction(np.array([0,1]), 'd')
LEFT  = Direction(np.array([-1,0]), 'l')
RIGHT = Direction(np.array([1,0]), 'r')
DIRECTIONS = np.array([UP, DOWN, LEFT, RIGHT])

def astar_search(init_state, goal_state, move_cost):
    start_state = State(np.array(init_state).reshape(3, 3), None)
    goal_state = State(np.array(goal_state).reshape(3, 3), None)

    # print(start_state.value)
    # print(goal_state.value)
    # print(start_state == goal_state)

    open = []
    closed = []
    # print(type(open), type(closed))
    open.append(start_state)

    while len(open) > 0:
    # for i in range(10):

        # Gets the state from the open list with the lowest f value
        current_state = open[0]
        current_index = 0
        for index, state in enumerate(open):
            if state.f < current_state.f:
                current_state = state
                current_index = index

        # Remove the current node from open list and add it to closed
        open.pop(current_index)
        closed.append(current_state)

        # If the current state is the goal state work backwards through children to find the optimal path
        if current_state == goal_state:
            moves = ""
            current = current_state
            while current is not None:
                if current.parent_move is not None: moves = moves + current.parent_move.char
                current = current.parent
            return moves[::-1]

        # Generate children
        children = []
        for move in valid_moves(current_state):
            pos_zero = get_pos(current_state, 0)
            possible_state = current_state.value.copy()
            # test2[1], test2[2] = test[2], test[1]
            possible_state[tuple(pos_zero)], possible_state[tuple(pos_zero + move.value)] = current_state.value[tuple(pos_zero + move.value)], current_state.value[tuple(pos_zero)]

            future_state_obj = State(possible_state, current_state, move)

            # print(future_state_obj.parent_move.char)
            children.append(future_state_obj)
            # future_state_obj.print()

        for child in children:
            # Check if child is in the closed list already
            for closed_child in closed:
                if child == closed_child: continue

            # Generate f, h, and g values. g values come from move_cost parameter
            child.g = current_state.g + 1
            child.h = h(child, goal_state)
            child.f = child.h + child.g

            for open_state in open:
                if child == open_state and child.g > open_state.g:
                    continue

            open.append(child)



    # test(start_state, goal_state)


def h(state, goal_state):
    total = 0
    for i in range(9):
        total = total + manhattan_distance(state, goal_state, i)
    return total

def manhattan_distance(state_a, state_b, value):
    return np.sum(np.absolute(get_pos(state_a, value) - get_pos(state_b, value)))

def get_pos(state, value):
    return np.argwhere(state.value == value)[0]

def valid_moves(state):
    valid_moves = []
    if get_pos(state, 0)[1] > 0:
        valid_moves.append(UP)
    if get_pos(state, 0)[1] < 2:
        valid_moves.append(DOWN)
    if get_pos(state, 0)[0] > 0:
        valid_moves.append(LEFT)
    if get_pos(state, 0)[0] < 2:
        valid_moves.append(RIGHT)
    return valid_moves

def test(start_state, goal_state):
    start_state.print()
    goal_state.print()
    print("get_pos")
    print(get_pos(start_state, 2))
    print(get_pos(goal_state, 2))
    print(get_pos(start_state, 7))
    print(get_pos(goal_state, 7))
    print(get_pos(start_state, 6))
    print(get_pos(goal_state, 6))
    print("manhattan_distance")
    print(manhattan_distance(start_state, goal_state, 2))
    print(manhattan_distance(start_state, goal_state, 7))
    print(manhattan_distance(start_state, goal_state, 6))
    print("heuristic")
    print(h(start_state, goal_state))

# 1 3 4
# 8 0 5
# 7 2 6

# 1 2 3
# 8 0 4
# 7 6 5
path = astar_search([1,8,7,3,0,2,4,5,6],[1,8,7,2,0,6,3,4,5],[1,1,1,1])
print(path)
