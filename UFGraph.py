from bridges.bridges import *
from bridges.data_src_dependent import *
from bridges.graph_adj_list import *
import heapq
import math

# class for  UF campus graph
class UFGraph:
    def __init__(self):
        # creates a bridges object
        self.graph = None
        self.bridges = Bridges(101, "JosephGuzman", "776326189690")
        self.bridges.set_title("UF Campus OSM Graph")
        self.bridges.set_description("Graph of the University of Florida campus")

        # bounding box around UF campus
        lat_min = 29.630
        long_min = -82.370
        lat_max = 29.660
        long_max = -82.320
        self.osmdata = data_source.get_osm_data(lat_min, long_min, lat_max, long_max, "default")

    # assign locations on campus to vertices, creates a graph and returns the graph object
    def makeGraph(self, osmdata):
        self.location_labels = {
            137: "Marston Science Library",
            491: "Library West",
            31: "Reitz Union",
            1630: "Gator Corner Dining",
            661: "Newell Hall",
            1126: "Norman Hall",
            97: "Pugh Hall",
            67: "New Physics Building",
            366: "Little Hall",
            1634: "Weil Hall",
            566: "Matherly Hall",
            347: "Heavener Hall",
            1636: "Hough Hall",
            1563: "Leigh Hall",
            968: "Grinter Hall",
            527: "Frazier Rogers Hall",
            122: "Floyd Hall",
            1638: "Fine Arts Building C",
            254: "Nanoscale Research Facility",
            253: "Biomedical Sciences Building",
            130: "Shands Hospital",
            2: "Ben Hill Griffin Stadium",
            1022: "Stephen C. O’Connell Center",
            994: "Condron Family Ballpark",
            478: "Beaty Towers",
            483: "Hume Hall",
            1664: "Jennings Hall",
            1080: "Broward Hall",
            453: "Rawlings Hall",
            678: "Graham Hall",
            714: "Keys Complex",
            738: "Southwest Recreation Center",
            105: "Student Recreation & Fitness Center",
            802: "Bat Houses"
        }

        vertices = osmdata.vertices
        edges = osmdata.edges

        # map vertex index to object
        vertex_lookup = {i: v for i, v in enumerate(vertices)}

        # the graph object
        graph = GraphAdjList()

        # build the graph from edges
        for e in edges:
            from_id = e._source
            to_id = e._destination
            weight = e._distance
            graph.add_vertex(from_id, str(from_id))
            if from_id in self.location_labels:
                graph.get_vertex(from_id).label = self.location_labels[from_id]

            # add and label to_id
            graph.add_vertex(to_id, str(to_id))
            if to_id in self.location_labels:
                graph.get_vertex(to_id).label = self.location_labels[to_id]

            # assign coordinates if found
            if from_id in vertex_lookup:
                v_from = vertex_lookup[from_id]
                graph.get_visualizer(from_id).set_location(v_from._latitude, v_from._longitude)

            if to_id in vertex_lookup:
                v_to = vertex_lookup[to_id]
                graph.get_visualizer(to_id).set_location(v_to._latitude, v_to._longitude)

            # Add edges both directions since roads go both ways
            graph.add_edge(to_id, from_id, weight)
            graph.add_edge(from_id, to_id, weight)

        self.bridges.set_data_structure(graph)
        self.bridges.visualize()
        return graph

    # colors a path on the graph given the graph and the path as parameters
    def colorPath(self, graph, path, vertex_color = "yellow", edge_color = "red", edge_thickness=2):

        # color each vertex in the path
        for v_id in path:
            graph.get_vertex(v_id).color = vertex_color

        # Color each edge in the path
        for i in range(len(path) - 1):
            vertexA = path[i]
            vertexB = path[i + 1]
            try:
                link_visualizer = graph.get_link_visualizer(vertexA, vertexB)
                link_visualizer.color = edge_color
                link_visualizer.thickness = edge_thickness
            except Exception as e:
                print(f"Could not color edge {vertexA} -> {vertexB}: {e}")

    # performs dijkstra's algorithm on the graph to find the most optimal route
    def dijkstrasAlgorithm(self, graph, source, target, osm_data):

        if not graph.get_vertex(source):
            return []
        if not graph.get_vertex(target):
            return []

        distance = {v: float('inf') for v in graph.key_set()}
        prev = {v: None for v in graph.key_set()}
        distance[source] = 0

        edge_lookup = {(e.source, e.destination): e for e in osm_data.edges}
        pq = [(0, source)]

        while pq:
            dist_u, u = heapq.heappop(pq)
            if u == target:
                break
            if dist_u > distance[u]:
                continue

            current = graph.get_adjacency_list(u)
            while current:
                edge = current.value
                v_id = edge.destination

                osm_edge = edge_lookup.get((u, v_id))
                if osm_edge is None:
                    current = current.next
                    continue

                weight = osm_edge.distance

                if distance[u] + weight < distance[v_id]:
                    distance[v_id] = distance[u] + weight
                    prev[v_id] = u
                    heapq.heappush(pq, (distance[v_id], v_id))

                current = current.next

        path = []
        curr = target
        while curr is not None:
            path.insert(0, curr)
            curr = prev[curr]
        if not path or path[0] != source:
            print("No path found")
            return []
        return path

    # A* algorithm to find the most optimal route in the graph
    def AStarAlgorithm(self, graph, source, target, osm_data):

        if not graph.get_vertex(source):
            return []
        if not graph.get_vertex(target):            return []

        open_set = []
        heapq.heappush(open_set, (0, source))

        came_from = {}
        g_score = {v: float('inf') for v in graph.key_set()}
        g_score[source] = 0

        f_score = {v: float('inf') for v in graph.key_set()}
        # Get target coordinates
        target_node = osm_data.get_vertex(target)
        target_lat = target_node.latitude
        target_lon = target_node.longitude

        # initial f_score
        source_node = osm_data.get_vertex(source)
        differenceInLatitude = source_node.latitude - target_lat
        differenceInLongitude = source_node.longitude - target_lon
        f_score[source] = math.sqrt(differenceInLatitude ** 2 + differenceInLongitude ** 2)

        # lookup for edge distances
        edge_lookup = {(e.source, e.destination): e for e in osm_data.edges}

        while open_set:
            x, current = heapq.heappop(open_set)

            if current == target:
                break

            current_list = graph.get_adjacency_list(current)
            while current_list:
                edge = current_list.value
                neighbor = edge.destination

                osm_edge = edge_lookup.get((current, neighbor), None)
                weight = osm_edge.distance

                tentative_g_score = g_score[current] + weight

                if tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score


                # get neighbor coordinates from visualizer
                    try:
                        neighbor_node = osm_data.get_vertex(neighbor)
                        dx = neighbor_node.latitude - target_lat
                        dy = neighbor_node.longitude - target_lon
                        heuristic = math.sqrt(dx * dx + dy * dy)

                    except Exception as e:
                        print(f"Couldn't get location for neighbor")
                        heuristic = 0

                    f_score[neighbor] = tentative_g_score + heuristic
                    heapq.heappush(open_set, (f_score[neighbor], neighbor))

                current_list = current_list.next

        # reconstruct path
        path = []
        curr = target
        while curr in came_from:
            path.insert(0, curr)
            curr = came_from[curr]
        if curr == source:
            path.insert(0, source)

        print(path)
        return path

    def findVertexfromCoordinates(self, lat, long, osm_data):
        closest_node = None
        min_dist = float('inf')

        for node_id in osm_data.get_vertices():
            node = osm_data.get_vertex(node_id)

            differenceInLatitude = node.latitude - lat
            differenceInLongitude = node.longitude - long
            dist = math.sqrt(differenceInLatitude * differenceInLongitude + differenceInLongitude * differenceInLongitude)

            if dist < min_dist:
                min_dist = dist
                closest_node = node_id

        return closest_node