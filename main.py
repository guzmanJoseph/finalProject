from UFGraph import UFGraph
from bridges.bridges import Bridges


def main():
    uf = UFGraph()
    graph = uf.makeGraph(uf.osmdata)

    # This is the template
    startingLatitude, startingLongitude = # getFromFrontEnd() get coordinates from the starting and end location
    endingLatitude, endingLongitude = # getFromFrontEnd() get coordinates from the starting and end location
    startingVertex = uf.findVertexfromCoordinates(startingLatitude, startingLongitude)
    endingVertex = uf.findVertexfromCoordinates(endingLatitude, endingLongitude)
    if(#user clicks == Djkstras):
        path = uf.dijkstrasAlgorithm(graph, startingVertex, endingVertex, uf.osm_data)
        uf.colorPath(graph, path)
        # return the link and display in front end
    else if(#user clicks == A-Star):
        uf.AstarAlgorithm(graph, startingVertex, endingVertex)
        uf.colorPath(graph,path)
        # return the link and display in front end


if __name__ == "__main__":
    main()
