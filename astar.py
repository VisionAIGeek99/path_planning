import matplotlib.pyplot as plt
import heapq
import time

class Node:
    """ A node class for A* Pathfinding """
    def __init__(self, parent=None, position=None):
        self.parent = parent
        self.position = position
        self.g = 0 # cost from start node
        self.h = 0 # heuristic cost to target node
        self.f = 0 # total cost

    def __eq__(self, other):
        return self.position == other.position

    def __lt__(self, other):
        return self.f < other.f

    def __hash__(self):
        return hash(self.position)

def heuristic(a, b):
    """ Calculate the heuristic based distance between two points """
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

def astar(grid, start, end, draw_callback):
    """ Performs the A* algorithm """
    open_list = []
    closed_set = set()
    start_node = Node(None, start)
    start_node.g = start_node.h = start_node.f = 0
    end_node = Node(None, end)
    end_node.g = end_node.h = end_node.f = 0

    heapq.heappush(open_list, (start_node.f, start_node))

    while open_list:
        current_node = heapq.heappop(open_list)[1]  # 비용이 가장 작은 노드를 openlist에서 꺼내기
        closed_set.add(current_node)

        if current_node == end_node:
            path = []
            while current_node is not None:
                path.append(current_node.position)
                current_node = current_node.parent
            return path[::-1]  # Return reversed path

        (x, y) = current_node.position
        neighbors = [(x-1, y), (x+1, y), (x, y-1), (x, y+1)]

        for next in neighbors:
            if next[0] > (len(grid) - 1) or next[0] < 0 or next[1] > (len(grid[0]) - 1) or next[1] < 0: # 격자내 존재 여부 검사
                continue
            if grid[next[0]][next[1]] != 0: # 장애물 검사
                continue

            neighbor = Node(current_node, next)
            if neighbor in closed_set:
                continue

            neighbor.g = current_node.g + 1
            neighbor.h = heuristic(neighbor.position, end_node.position)
            neighbor.f = neighbor.g + neighbor.h

            if add_to_open(open_list, neighbor):
                heapq.heappush(open_list, (neighbor.f, neighbor))

        draw_callback(grid, start, end, path=[], current=current_node.position, open_set=[node.position for f, node in open_list], closed_set=[node.position for node in closed_set])
        time.sleep(0.5)  # Delay for visualization

def add_to_open(open_list, neighbor):
    """ Check if a neighbor should be added to open list """
    for node in open_list:
        if neighbor == node[1] and neighbor.g > node[1].g: # open_list에 존재하고 기존 노드보다 더 나은 경로가 있는지 검사
            return False
    return True

def draw_grid(grid, start, end, path, current, open_set, closed_set):
    """ Draw the grid and the pathfinding process """
    plt.cla()  # Clear the current axes
    plt.imshow(grid, cmap='hot', interpolation='nearest')
    for y in range(len(grid)):
        for x in range(len(grid[0])):
            if (y, x) == start:
                plt.text(x, y, 'S', ha='center', va='center', color='white')
            elif (y, x) == end:
                plt.text(x, y, 'E', ha='center', va='center', color='white')
            elif (y, x) in path:
                plt.scatter(x, y, color='blue', s=100)
            elif (y, x) == current:
                plt.scatter(x, y, color='red', s=100)
            elif (y, x) in open_set:
                plt.scatter(x, y, color='green', s=50)
            elif (y, x) in closed_set:
                plt.scatter(x, y, color='yellow', s=50)
    plt.pause(0.2)  # Pause to update the plot

if __name__ == "__main__":
    # Example grid and start/end points
    grid = [[0, 0, 0, 1, 0, 0, 0, 0],
            [0, 1, 0, 1, 0, 1, 1, 0],
            [0, 1, 0, 1, 0, 0, 0, 0],
            [0, 0, 0, 1, 1, 0, 0, 0],
            [0, 1, 0, 0, 0, 0, 1, 0],
            [0, 0, 0, 1, 1, 1, 1, 0]]
    start = (2, 0)
    end = (5, 7)

    # Run the A* algorithm with visualization
    path = astar(grid, start, end, draw_grid)

    # Draw final path
    draw_grid(grid, start, end, path, current=None, open_set=[], closed_set=[])

    plt.show()
# Red Node: Current Node
# Green Node: Noeds in the OpenList
# Red Node: Noeds in the ClosedList
# Blue Node: Nodes belonging to the final path