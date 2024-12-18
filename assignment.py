# set graph representation

class Graph:
    def __init__(self):
        self.vertices = {}

    def add_vertex(self, vertex):
        if vertex not in self.vertices:
            self.vertices[vertex] = {}

    def add_edge(self, source, destination, weight):
        if source in self.vertices and destination in self.vertices:
            self.vertices[source][destination] = weight
            self.vertices[destination][source] = weight  # For undirected graph

    def get_neighbors(self, vertex):
        if vertex in self.vertices:
            return self.vertices[vertex]
        else:
            return {}

# use dijkstra's algorithm

def dijkstra(graph, start):
    import heapq

    distances = {vertex: float('inf') for vertex in graph.vertices}
    previous_vertices = {vertex: None for vertex in graph.vertices}
    distances[start] = 0
    priority_queue = [(0, start)]

    while priority_queue:
        current_distance, current_vertex = heapq.heappop(priority_queue)

        if current_distance > distances[current_vertex]:
            continue

        for neighbor, weight in graph.get_neighbors(current_vertex).items():
            distance = current_distance + weight

            if distance < distances[neighbor]:
                distances[neighbor] = distance
                previous_vertices[neighbor] = current_vertex
                heapq.heappush(priority_queue, (distance, neighbor))

    return distances, previous_vertices

# test algorithm implementation

def print_shortest_paths(distances, previous_vertices, start):
    for vertex, distance in distances.items():
        if distance == float('inf'):
            print(f"No path from {start} to {vertex}.")
        else:
            path = []
            current = vertex
            while current is not None:
                path.append(current)
                current = previous_vertices[current]
            path.reverse()
            print(f"Shortest path from {start} to {vertex}: {' -> '.join(path)} with distance {distance}")

# Create a graph instance
graph = Graph()
graph.add_vertex('A')
graph.add_vertex('B')
graph.add_vertex('C')
graph.add_vertex('D')

graph.add_edge('A', 'B', 5)
graph.add_edge('B', 'C', 3)
graph.add_edge('A', 'C', 10)
graph.add_edge('B', 'D', 2)

distances, previous_vertices = dijkstra(graph, 'A')
print_shortest_paths(distances, previous_vertices, 'A')

# Task 4: Analyze Time and Space Complexity

# Time Complexity: O((V + E) log V)
# - V: Number of vertices in the graph
# - E: Number of edges in the graph
# Space Complexity: O(V + E)
# - Storing distances, previous_vertices, and adjacency list

print("\nTime Complexity: O((V + E) log V)")
print("Space Complexity: O(V + E)")
