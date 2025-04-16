from bridges.bridges import *
from bridges.data_src_dependent import data_source
from bridges.graph_adj_list import GraphAdjList

def main():
    print("Fetching OSM data for UF...")

    bridges = Bridges(999, "JosephGuzman", "776326189690")
    bridges.set_title("UF Campus OSM Graph")
    bridges.set_description("OpenStreetMap graph of the University of Florida campus")

    # Bounding box around UF campus
    lat_min = 29.630
    long_min = -82.370
    lat_max = 29.660
    long_max = -82.320

    # Fetch OSM data
    osmdata = data_source.get_osm_data(lat_min, long_min, lat_max, long_max, "default")
    vertices = osmdata.vertices
    edges = osmdata.edges

    print(f"Fetched {len(vertices)} vertices and {len(edges)} edges")
    print("Edge sample:", vars(edges[0]))  # You can now remove this line if everything works

    # Use list index as vertex ID
    vertex_lookup = {}
    for i, v in enumerate(vertices):
        vertex_lookup[i] = v

    graph = GraphAdjList()

    for e in edges:
        from_id = e._source
        to_id = e._destination
        weight = e._distance

        graph.add_vertex(from_id, str(from_id))
        graph.add_vertex(to_id, str(to_id))

        if from_id in vertex_lookup:
            v_from = vertex_lookup[from_id]
            graph.get_visualizer(from_id).set_location(v_from._latitude, v_from._longitude)

        if to_id in vertex_lookup:
            v_to = vertex_lookup[to_id]
            graph.get_visualizer(to_id).set_location(v_to._latitude, v_to._longitude)

        graph.add_edge(from_id, to_id, weight)
        graph.add_edge(to_id, from_id, weight)

    bridges.set_data_structure(graph)
    bridges.visualize()

if __name__ == "__main__":
    main()