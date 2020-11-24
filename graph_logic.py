import heapq


class Node:
    def __init__(self, vertex, distance):
        self.distance = distance
        self.vertex = vertex

    def __cmp__(self, other):
        return self.distance - other.distance

    def __lt__(self, other):
        return self.__cmp__(other) < 0

    def __le__(self, other):
        return self.__cmp__(other) <= 0

    def __gt__(self, other):
        return self.__cmp__(other) > 0

    def __ge__(self, other):
        return self.__cmp__(other) >= 0

    def __eq__(self, other):
        return self.vertex == other.vertex


class Graph:

    def __init__(self, nodes: list, locations: list, edges: dict, root: int = None):
        self.nodes = nodes
        self.locations = locations
        self.edges: dict = edges
        self.root = root

    def get_neighbors(self, node_index: int) -> dict:
        return self.edges.get(node_index, {})

    def ucs(self, start: int = None):
        if start is None:
            start = self.root
        distances = {}
        processed = set()
        prev = {}

        distances[start] = 0
        fringe = []
        heapq.heappush(fringe, (distances[start], start))

        prev[start] = start

        while len(fringe) != 0:
            distance, vertex = heapq.heappop(fringe)
            if vertex not in processed:
                for child, weight in self.get_neighbors(vertex).items():
                    if distance + weight < distances.get(child, float('inf')):
                        if (distances.get(child, float('inf')), child) in fringe:
                            fringe.remove((distances.get(child, float('inf')), child))
                        distances[child] = distance + weight
                        heapq.heappush(fringe, (distances[child], child))
                        prev[child] = vertex
                processed.add(vertex)

        return prev

    def hypotenuse(self, a, b):
        x = self.locations[a][0] - self.locations[b][0]
        y = self.locations[a][1] - self.locations[b][1]
        z = self.locations[a][2] - self.locations[b][2]
        return (x ** 2 + y ** 2 + z ** 2) ** 0.5

    def astar(self, goal: int, start: int = None):

        if start is None:
            start = self.root
        gscore = {start: 0}
        fscore = {start: self.hypotenuse(start, goal)}
        processed = set()
        prev = {}

        fringe = []
        heapq.heappush(fringe, (fscore[start], start))

        prev[start] = None

        while len(fringe) != 0:
            distance, current = heapq.heappop(fringe)
            if current == goal:
                path = []
                while current in prev:
                    path.append(current)
                    current = prev[current]
                path.reverse()
                return path
            processed.add(current)
            for neighbor, weight in self.get_neighbors(current).items():
                tentative_g_score = gscore[current] + self.hypotenuse(current, neighbor)

                if neighbor in processed and tentative_g_score >= gscore.get(neighbor, 0):
                    continue

                if neighbor not in gscore or tentative_g_score < gscore[neighbor]:
                    prev[neighbor] = current
                    gscore[neighbor] = tentative_g_score
                    fscore[neighbor] = tentative_g_score + self.hypotenuse(neighbor, goal)
                    heapq.heappush(fringe, (fscore[neighbor], neighbor))

        return None

    def get_components(self):
        seen = {}
        for i, location in enumerate(self.locations):
            if i in seen:
                continue
            stack = [i]
            island = set()
            while len(stack) > 0:
                current = stack.pop()
                if current in island:
                    continue
                island.add(current)
                if current in seen:
                    island.update(seen[current])
                for neighbor in self.get_neighbors(current).keys():
                    stack.append(neighbor)
            for node in island:
                seen[node] = island
        ret = set()
        for value in seen.values():
            ret.add(tuple(value))
        return ret
