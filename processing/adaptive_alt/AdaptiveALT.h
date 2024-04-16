//
// Created by milos on 09/04/2024.
//

#ifndef ND_KNN_ADAPTIVEALT_H
#define ND_KNN_ADAPTIVEALT_H


#include "../Graph.h"
#include "../Path.h"
#include "../ier/ObjectList.h"
#include "../../queue/BinaryMinHeap.h"
#include "../DijkstraSearch.h"

#include <boost/archive/binary_oarchive.hpp>
#include <boost/serialization/vector.hpp>
#include <set>

struct Landmark {
    unsigned int index;
    NodeID nodeId;
};

class AdaptiveALT {

public:
    AdaptiveALT(int numNodes, int numEdges, int maxNumLandmarks);
    bool shouldCreateLandmark(NodeID node);
    PathDistance findShortestPathDistance(Graph &graph, NodeID source, NodeID target);

    EdgeWeight getLowerBound(NodeID s, NodeID t);

    void deleteLowestScoreLandmark();

    unsigned long getEdgesAccessedCount()
    {
        return edgesAccessedCount + dijkstra.edgesAccessedCount;
    }

    void printStatistics() {
//        for(unsigned i = 0; i < landmarks.size(); ++i) {
//            std::cout << "Landmark " << i << " score: " << landmarkScore(i) << std::endl;
//        }
    }

private:
    unsigned int numNodes;
    unsigned int numEdges;
    unsigned int maxNumLandmarks;
    unsigned int numLandmarks;

    std::vector<Landmark> landmarks;
    std::vector<EdgeWeight> landmarksMaxDistances;
//    std::vector<unsigned> landmarksQueryAnswered;
//    std::vector<unsigned> landmarksQueryNumber;
    std::vector<int> vertexFromLandmarkDistances;
    DijkstraSearch dijkstra;

    // temporary variables
    std::vector<EdgeWeight> landmarkDistances;
    BinaryMinHeap<EdgeWeight, NodeID> pqueue;

    unsigned createLandmark(Graph &graph, NodeID node);
    PathDistance shortestPathDistanceALT(Graph &graph, NodeID source, NodeID target);

//    inline double landmarkScore(unsigned landmark) {
//        if(landmarksQueryNumber[landmark] == 0) {
//            return 0;
//        }
//        return landmarksQueryAnswered[landmark] / (double)landmarksQueryNumber[landmark];
//    }

    double closestLandmarkQuality(NodeID node);

    inline EdgeWeight nodeFromLandmarkDistance(unsigned landmarkIndex, NodeID node) {
        return vertexFromLandmarkDistances[node * maxNumLandmarks + landmarkIndex];
    }

    // *Stats*
    std::set<int> edgesAccessed;
    unsigned long edgesAccessedCount = 0;
    inline EdgeWeight getEdgeWeight(Graph &graph, int i)
    {
//        edgesAccessed.insert(i);
        return graph.edges[i].second;
    }
};


#endif //ND_KNN_ADAPTIVEALT_H
