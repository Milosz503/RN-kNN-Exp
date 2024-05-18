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
//#define ESTIMATE_VISITED_NODES 1
//#define USE_EUCLIDEAN 1

struct AdaptiveALTParams {
    AdaptiveALTParams(const unsigned int maxLandmarks, const double a, const double b, const double c,
                      const std::function<double(unsigned int)> &thresholdFunction) :
            maxLandmarks(maxLandmarks),
            a(a),
            b(b),
            c(c),
            thresholdFunction(thresholdFunction)
    {}

    AdaptiveALTParams(const unsigned int maxLandmarks, const double a, const double b, const double c,
                      double threshold) :
            AdaptiveALTParams(maxLandmarks, a, b, c, [threshold](unsigned _) { return threshold; })
    {}

    const unsigned maxLandmarks;
    const double a;
    const double b;
    const double c;
    std::function<double(unsigned)> thresholdFunction;
};

struct Landmark {
    unsigned int index;
    NodeID nodeId;
#ifdef DYNAMIC_LANDMARKS_A_ALT
    std::vector<EdgeWeight> distances;
#endif
    std::vector<unsigned> pathLengths;

#ifdef ESTIMATE_VISITED_NODES
    std::vector<unsigned> nodesVisited;
#endif

    Landmark(unsigned int index, NodeID node, unsigned int numNodes) :
            index(index),
            nodeId(node),
#ifdef DYNAMIC_LANDMARKS_A_ALT
            distances(numNodes, 0),
#endif
#ifdef ESTIMATE_VISITED_NODES
            nodesVisited(numNodes, 0),
#endif
            pathLengths(numNodes, 0)

    {}
};

class AdaptiveALT {

public:
    AdaptiveALT(int numNodes, int numEdges, AdaptiveALTParams &params);

    PathDistance findShortestPathDistance(Graph &graph, NodeID source, NodeID target);

    EdgeWeight getLowerBound(Graph& graph, NodeID s, NodeID t);

    EdgeWeight getLowerBound(NodeID s, NodeID t, std::vector<unsigned> &landmarkIndexes);

    unsigned long getEdgesAccessedCount()
    {
        return edgesAccessedCount + dijkstra.edgesAccessedCount;
    }

    void printInfo()
    {
        std::cout << "a=" << params.a << ", b=" << params.b << ", c=" << params.c
                  << ", threshold(0)=" << params.thresholdFunction(0)
                  << ", threshold(4096)=" << params.thresholdFunction(4096) << std::endl;
    }

    void printStatistics()
    {
//        for(unsigned i = 0; i < landmarks.size(); ++i) {
//            unsigned landmark = landmarks[i].index;
//            std::cout << "Landmark " << landmark << " score: " << landmarkScore(landmark) << std::endl;
//        }
//        std::cout << "Number of landmarks: " << landmarks.size() << std::endl;
//        std::cout << "Score time: " << scoreTime << std::endl;
#ifdef USE_EUCLIDEAN
        std::cout << ", " << landmarks.size() << ", " << euclideanCounter / (double)(euclideanCounter + landmarkCounter);
#else
        std::cout << ", " << landmarks.size();
#endif
        std::cout << ", " << landmarkCreateTime;
    }

private:
    unsigned int numNodes;
    unsigned int numEdges;
    unsigned int maxNumLandmarks;
    unsigned int numLandmarks;
    const AdaptiveALTParams params;

    unsigned int queryNumber = 0;
    double cumulativeEstimateError = 0;
    double estimationCount = 0;
    double landmarkCreateTime = 0;

    unsigned euclideanCounter = 0;
    unsigned landmarkCounter = 0;

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
    std::vector<EdgeWeight> tempLandmarkDistances;

    unsigned createLandmark(Graph &graph, NodeID node);

    PathDistance shortestPathDistanceALT(Graph &graph, NodeID source, NodeID target);

    inline double landmarkScore(unsigned landmark)
    {
        if (landmarksQueryNumber[landmark] == 0) {
            return 0;
        }
        return landmarksQueryAnswered[landmark] / (double) landmarksQueryNumber[landmark];
    }

    double closestLandmarkDistanceRatio(NodeID node);

    double closestLandmarkNodesRatio(NodeID node);

    unsigned findClosestLandmark(NodeID t);

    double estimatePathLengthRatio(NodeID s, NodeID t);

    inline EdgeWeight nodeFromLandmarkDistance(unsigned landmarkIndex, NodeID node)
    {
#ifdef DYNAMIC_LANDMARKS_A_ALT
        return landmarks[landmarkIndex].distances[node];
#else
        return vertexFromLandmarkDistances[node * maxNumLandmarks + landmarkIndex];
#endif
    }

    std::vector<unsigned> selectBestLandmarks(NodeID s, NodeID t);

    // *Stats*
    std::set<int> edgesAccessed;
    unsigned nodesVisited;
    unsigned long edgesAccessedCount = 0;

    inline EdgeWeight getEdgeWeight(Graph &graph, int i)
    {
//        edgesAccessed.insert(i);
        return graph.edges[i].second;
    }
};


#endif //ND_KNN_ADAPTIVEALT_H
