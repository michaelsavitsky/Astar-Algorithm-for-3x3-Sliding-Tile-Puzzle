import numpy as np
import time

class Direction(object):
    def __init__(self, value, char, move_cost):
        self.value = value
        self.char = char
        self.move_cost = move_cost
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
        print("|",self.value[0,0], self.value[1,0], self.value[2,0],"|")
        print("|",self.value[0,1], self.value[1,1], self.value[2,1],"|")
        print("|",self.value[0,2], self.value[1,2], self.value[2,2],"|")
        print("----------")
    def __eq__(self, other):
        return (self.value == other.value).all()

def astar_search(init_state, goal_state, move_cost):
    start_state = State(np.array(init_state).reshape(3, 3), None)
    goal_state = State(np.array(goal_state).reshape(3, 3), None)

    UP    = Direction(np.array([0,-1]), 'u', move_cost[0])
    DOWN  = Direction(np.array([0,1]), 'd', move_cost[1])
    LEFT  = Direction(np.array([-1,0]), 'l', move_cost[2])
    RIGHT = Direction(np.array([1,0]), 'r', move_cost[3])
    DIRECTIONS = np.array([UP, DOWN, LEFT, RIGHT])

    open = []
    closed = []
    open.append(start_state)

    while len(open) > 0:

        # Gets the state from the open list with the lowest f value
        current_state = open.pop(0)

        # Add it to closed
        closed.append(current_state)

        # If the current state is the goal state work backwards through children to find the optimal path
        if current_state == goal_state:
            moves = ""
            current = current_state
            total_cost = 0
            while current is not None:
                if current.parent_move is not None:
                    moves = moves + current.parent_move.char
                    total_cost = total_cost + current.parent_move.move_cost
                current = current.parent
            print("Total Cost:", total_cost)
            return moves[::-1]

        # Get list of valid moves given current state
        valid_moves = []
        if (current_state.parent_move is None):
            if get_pos(current_state, 0)[1] > 0:
                valid_moves.append(UP)
            if get_pos(current_state, 0)[1] < 2:
                valid_moves.append(DOWN)
            if get_pos(current_state, 0)[0] > 0:
                valid_moves.append(LEFT)
            if get_pos(current_state, 0)[0] < 2:
                valid_moves.append(RIGHT)
        else:
            if get_pos(current_state, 0)[1] > 0 and current_state.parent_move != DOWN:
                valid_moves.append(UP)
            if get_pos(current_state, 0)[1] < 2 and current_state.parent_move != UP:
                valid_moves.append(DOWN)
            if get_pos(current_state, 0)[0] > 0 and current_state.parent_move != RIGHT:
                valid_moves.append(LEFT)
            if get_pos(current_state, 0)[0] < 2 and current_state.parent_move != LEFT:
                valid_moves.append(RIGHT)

        # Generate children
        children = []
        for move in valid_moves:
            pos_zero = get_pos(current_state, 0)
            possible_state = current_state.value.copy()
            # Swap two tiles
            possible_state[tuple(pos_zero)], possible_state[tuple(pos_zero + move.value)] = current_state.value[tuple(pos_zero + move.value)], current_state.value[tuple(pos_zero)]

            # Create State obj
            child = State(possible_state, current_state, move)

            # Append to children
            children.append(child)

        for future_child in children:

            for closed_child in closed:
                if future_child == closed_child: continue

            future_child.g = current_state.g + move.move_cost
            future_child.h = h(future_child, goal_state)
            future_child.f = future_child.h + future_child.g

            for open_state in open:
                if future_child == open_state:
                    if future_child.g >= open_state.g:
                        # Already there with a smaller g value
                        continue
                    if future_child.g < open_state.g:
                        open_state.g = future_child.g

            open.append(future_child)
        open.sort(key=lambda child: child.f)
        # print("size", len(open))
        # print("f lowest:", open[0].f)
        # print("f highest:", open[len(open) - 1].f)
        # if (open[0].g > 100): open[0].print()
        # time.sleep(0.05)


# Calculates heuristic for a given state
def h(current_state, goal_state):
    total = 0
    for i in range(9):
        total = total + np.sum(np.absolute(get_pos(current_state, i) - get_pos(goal_state, i)))
    return total

# Gets the x,y position of a given value in a state
def get_pos(state, value):
    return np.argwhere(state.value == value)[0]

# [0,8,7,1,2,6,3,4,5], [1,8,7,2,0,6,3,4,5], [1,1,1,1], 'rd', 2
# [1,8,7,3,0,2,4,5,6], [1,8,7,2,0,6,3,4,5], [1,1,1,1], 'druuld', 6
# [1,6,0,2,7,8,3,4,5], [1,8,7,2,0,6,3,4,5], [1,1,2,2], 'urdlur', 9
# [1,6,0,2,7,8,3,4,5], [1,8,7,2,0,6,3,4,5], [3,3,1,1], 'ruldru', 12
# [8,0,7,1,4,3,2,5,6], [1,8,7,2,0,6,3,4,5], [1,1,1,1], 'urddrulurdl', 11
# [8,0,7,1,4,3,2,5,6], [1,8,7,2,0,6,3,4,5], [1,1,2,2], 'urddrulurdl', 16

print(astar_search([0,8,7,1,2,6,3,4,5], [1,8,7,2,0,6,3,4,5], [1,1,1,1]))
print(astar_search([1,8,7,3,0,2,4,5,6], [1,8,7,2,0,6,3,4,5], [1,1,1,1]))
print(astar_search([1,6,0,2,7,8,3,4,5], [1,8,7,2,0,6,3,4,5], [1,1,2,2]))
print(astar_search([1,6,0,2,7,8,3,4,5], [1,8,7,2,0,6,3,4,5], [3,3,1,1]))
print(astar_search([8,0,7,1,4,3,2,5,6], [1,8,7,2,0,6,3,4,5], [1,1,1,1]))
print(astar_search([8,0,7,1,4,3,2,5,6], [1,8,7,2,0,6,3,4,5], [1,1,2,2]))
print(astar_search([1,8,3,7,0,5,6,2,4], [1,4,7,2,5,8,3,6,0], [1,1,1,1]))
