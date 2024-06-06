import heapq

class Node:
    def __init__(self, state, cost, parent=None):
        self.state = state
        self.cost = cost
        self.parent = parent

    def __lt__(self, other):
        return self.cost < other.cost

def uniform_cost_search(initial_state, goal_state, get_neighbors, get_cost):
    # Priority queue to store nodes with the lowest cost at the front
    frontier = []
    heapq.heappush(frontier, Node(initial_state, 0))

    # Set to keep track of visited states
    visited = set()

    while frontier:
        current_node = heapq.heappop(frontier)

        if current_node.state == goal_state:
            return reconstruct_path(current_node)

        visited.add(current_node.state)

        neighbors = get_neighbors(current_node.state)
        for neighbor_state in neighbors:
            if neighbor_state not in visited:
                cost = current_node.cost + get_cost(current_node.state, neighbor_state)
                heapq.heappush(frontier, Node(neighbor_state, cost, current_node))

    return None  # If goal state is not reachable

def reconstruct_path(node):
    path = []
    while node:
        path.insert(0, node.state)
        node = node.parent
    return path

# Example usage:
def get_neighbors(state):
    # Define how to get neighbors for a given state
    # This is a placeholder function, replace it with your own logic
    pass

def get_cost(current_state, next_state):
    # Define the cost function between two states
    # This is a placeholder function, replace it with your own logic
    pass

# Example:
initial_state = "A"
goal_state = "D"

# Example implementation of get_neighbors and get_cost functions
def get_neighbors(state):
    neighbors = {"A": ["B", "C"],
                 "B": ["A", "D"],
                 "C": ["A", "D"],
                 "D": ["B", "C"]}
    return neighbors[state]

def get_cost(current_state, next_state):
    costs = {("A", "B"): 1, ("A", "C"): 2, ("B", "D"): 3, ("C", "D"): 1}
    return costs.get((current_state, next_state), float('inf'))

path = uniform_cost_search(initial_state, goal_state, get_neighbors, get_cost)

if path:
    print("Uniform Cost Search Path:", path)
else:
    print("Goal state is not reachable.")