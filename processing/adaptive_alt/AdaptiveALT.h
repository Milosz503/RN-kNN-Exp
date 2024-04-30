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

//#define DYNAMIC_LANDMARKS_A_ALT 1


struct Landmark {
    unsigned int index;
    NodeID nodeId;
#ifdef DYNAMIC_LANDMARKS_A_ALT
    std::vector<EdgeWeight> distances;
#endif
    std::vector<unsigned> pathLengths;
//    boost::unordered_map<NodeID, EdgeWeight> distances;

    Landmark(unsigned int index, NodeID node, unsigned int numNodes) :
        index(index),
        nodeId(node),
#ifdef DYNAMIC_LANDMARKS_A_ALT
        distances(numNodes, 0),
#endif
        pathLengths(numNodes, 0)
    {}
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

    void printInfo() {
        std::cout << "a=" << a << ", b=" << b << ", c=" << c << ", threshold=" << threshold << std::endl;
    }
    void printStatistics() {
//        for(unsigned i = 0; i < landmarks.size(); ++i) {
//            unsigned landmark = landmarks[i].index;
//            std::cout << "Landmark " << landmark << " score: " << landmarkScore(landmark) << std::endl;
//        }
        std::cout << "Number of landmarks: " << landmarks.size() << std::endl;

    }

private:
    const double a;
    const double b;
    const double c;
    const double threshold;

    unsigned int numNodes;
    unsigned int numEdges;
    unsigned int maxNumLandmarks;
    unsigned int numLandmarks;

    std::vector<Landmark> landmarks;
#ifndef DYNAMIC_LANDMARKS_A_ALT
    std::vector<unsigned int> vertexFromLandmarkDistances;
#endif
    std::vector<EdgeWeight> landmarksMaxDistances;
    std::vector<unsigned> landmarksMaxPaths;
    std::vector<unsigned> landmarksQueryAnswered;
    std::vector<unsigned> landmarksQueryNumber;
    DijkstraSearch dijkstra;

    // temporary variables
    BinaryMinHeap<EdgeWeight, NodeData> tempPqueue;

    unsigned createLandmark(Graph &graph, NodeID node);
    PathDistance shortestPathDistanceALT(Graph &graph, NodeID source, NodeID target);

    inline double landmarkScore(unsigned landmark) {
        if(landmarksQueryNumber[landmark] == 0) {
            return 0;
        }
        return landmarksQueryAnswered[landmark] / (double)landmarksQueryNumber[landmark];
    }

    double closestLandmarkDistanceRatio(NodeID node);
    double closestLandmarkNodesRatio(NodeID node);
    double estimatePathLengthRatio(NodeID s, NodeID t);

    inline EdgeWeight nodeFromLandmarkDistance(unsigned landmarkIndex, NodeID node) {
#ifdef DYNAMIC_LANDMARKS_A_ALT
        return landmarks[landmarkIndex].distances[node];
#else
        return vertexFromLandmarkDistances[node * maxNumLandmarks + landmarkIndex];
#endif
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
