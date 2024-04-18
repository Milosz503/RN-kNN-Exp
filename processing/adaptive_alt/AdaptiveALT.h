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
#include <boost/unordered_map.hpp>

struct Landmark {
    unsigned int index;
    NodeID nodeId;
    std::vector<EdgeWeight> distances;
//    boost::unordered_map<NodeID, EdgeWeight> distances;
};

class AdaptiveALT {

public:
    AdaptiveALT(int numNodes, int numEdges, int maxNumLandmarks);
    PathDistance findShortestPathDistance(Graph &graph, NodeID source, NodeID target);

    EdgeWeight getLowerBound(NodeID s, NodeID t);
    EdgeWeight getLowerBound(NodeID s, NodeID t, std::vector<unsigned>& landmarkIndexes);

    unsigned long getEdgesAccessedCount()
    {
        return edgesAccessedCount + dijkstra.edgesAccessedCount;
    }

    void printStatistics() {
//        for(unsigned i = 0; i < landmarks.size(); ++i) {
//            unsigned landmark = landmarks[i].index;
//            std::cout << "Landmark " << landmark << " score: " << landmarkScore(landmark) << std::endl;
//        }
        std::cout << "Number of landmarks: " << landmarks.size() << std::endl;
    }

private:
    unsigned int numNodes;
    unsigned int numEdges;
    unsigned int maxNumLandmarks;
    unsigned int numLandmarks;

    std::vector<Landmark> landmarks;
    std::vector<EdgeWeight> landmarksMaxDistances;
    std::vector<unsigned> landmarksQueryAnswered;
    std::vector<unsigned> landmarksQueryNumber;
    DijkstraSearch dijkstra;

    // temporary variables
    std::vector<EdgeWeight> tempLandmarkDistances;
    BinaryMinHeap<EdgeWeight, NodeID> tempPqueue;

    unsigned createLandmark(Graph &graph, NodeID node);
    PathDistance shortestPathDistanceALT(Graph &graph, NodeID source, NodeID target);

    inline double landmarkScore(unsigned landmark) {
        if(landmarksQueryNumber[landmark] == 0) {
            return 0;
        }
        return landmarksQueryAnswered[landmark] / (double)landmarksQueryNumber[landmark];
    }

    double closestLandmarkQuality(NodeID node);

    inline EdgeWeight nodeFromLandmarkDistance(unsigned landmarkIndex, NodeID node) {
        return landmarks[landmarkIndex].distances[node];
    }

    std::vector<unsigned> selectBestLandmarks(NodeID s, NodeID t);

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
