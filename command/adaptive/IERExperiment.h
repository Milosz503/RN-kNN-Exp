//
// Created by milos on 08/04/2024.
//

#ifndef ND_KNN_IEREXPERIMENT_H
#define ND_KNN_IEREXPERIMENT_H

#include "Experiment.h"

class IERExperiment : public Experiment {
public:

    explicit IERExperiment(unsigned int branchFactor) :
            branchFactor(branchFactor),
            rtree(nullptr)
    {}

    void buildIndex(Graph &graph) override
    {

    }

    void
    loadObjects(Graph &graph, std::vector<NodeID>& objectNodes) override
    {
//        std::cout << "Coordinates ######################\n\n\n\n" << std::endl;
        std::vector<CoordinatePair> objectCoords;
        for (std::size_t i = 0; i < objectNodes.size(); ++i) {
            CoordinatePair objectCoordPair;
            graph.getCoordinates(objectNodes[i],objectCoordPair.first,objectCoordPair.second);
            objectCoords.push_back(objectCoordPair);
//            std::cout << objectCoordPair.first << ", " << objectCoordPair.second << std::endl;
        }
//        std::cout << "Coordinates end ######################\n\n\n\n" << std::endl;

        rtree = new StaticRtree(branchFactor);
        rtree->bulkLoad(objectNodes, objectCoords);
    }

    void clearObjects() override
    {
        delete rtree;
        rtree = nullptr;
    }

    void runQuery(Graph &graph, unsigned int k, NodeID queryNodeID, std::vector<NodeID> &kNNs,
                  std::vector<EdgeWeight> &kNNDistances) override
    {
        queries++;
        ier.getKNNsByDijkstra(*rtree, k, queryNodeID, kNNs, kNNDistances, graph);
    }

    std::string getName() override
    {
        return "IER-Dijkstra";
    }

    void printInfo() override
    {
//        std::cout << "fanout: " << fanout << ", maxLeafSize: " << maxLeafSize << ", levels: " << agtree->getNumLevels()
//                  << std::endl;
    }

    void printSummary() override
    {
        std::cout << "Avg candidates: " << ier.numCandidates / (float) queries << std::endl;
        std::cout << "Avg distance: " << ier.distanceSum / (double) ier.numCandidates << std::endl;
        std::cout << "Edges accessed: " << ier.edgesAccessedCount << std::endl;
    }

private:
    unsigned int branchFactor;
    IER ier;
    StaticRtree* rtree;
    int queries = 0;
};

class IERAStarExperiment : public Experiment {
public:

    explicit IERAStarExperiment(unsigned int branchFactor) :
            branchFactor(branchFactor),
            rtree(nullptr)
    {}

    void buildIndex(Graph &graph) override
    {

    }

    void
    loadObjects(Graph &graph, std::vector<NodeID>& objectNodes) override
    {
        std::vector<CoordinatePair> objectCoords;
        for (std::size_t i = 0; i < objectNodes.size(); ++i) {
            CoordinatePair objectCoordPair;
            graph.getCoordinates(objectNodes[i],objectCoordPair.first,objectCoordPair.second);
            objectCoords.push_back(objectCoordPair);
        }

        rtree = new StaticRtree(branchFactor);
        rtree->bulkLoad(objectNodes, objectCoords);
    }

    void clearObjects() override
    {
        delete rtree;
        rtree = nullptr;
    }

    void runQuery(Graph &graph, unsigned int k, NodeID queryNodeID, std::vector<NodeID> &kNNs,
                  std::vector<EdgeWeight> &kNNDistances) override
    {
        ier.getKNNsByAStar(*rtree, k, queryNodeID, kNNs, kNNDistances, graph);
    }

    std::string getName() override
    {
        return "IER-AStar";
    }

    void printInfo() override
    {
//        std::cout << "fanout: " << fanout << ", maxLeafSize: " << maxLeafSize << ", levels: " << agtree->getNumLevels()
//                  << std::endl;
    }

private:
    unsigned int branchFactor;
    IER ier;
    StaticRtree* rtree;
};

class IERALTExperiment : public Experiment {
public:

    explicit IERALTExperiment(unsigned int branchFactor, unsigned int numLandmarks, LANDMARK_TYPE landmarkType, ALTParameters params) :
            branchFactor(branchFactor),
            numLandmarks(numLandmarks),
            rtree(nullptr),
            landmarkType(landmarkType),
            alt("", 0, 0, params)
    {}

    void buildIndex(Graph &graph) override
    {
    }

    void
    loadObjects(Graph &graph, std::vector<NodeID>& objectNodes) override
    {
        alt.buildALT(graph, objectNodes, landmarkType, numLandmarks);

        std::vector<CoordinatePair> objectCoords;
        for (std::size_t i = 0; i < objectNodes.size(); ++i) {
            CoordinatePair objectCoordPair;
            graph.getCoordinates(objectNodes[i],objectCoordPair.first,objectCoordPair.second);
            objectCoords.push_back(objectCoordPair);
        }

        rtree = new StaticRtree(branchFactor);
        rtree->bulkLoad(objectNodes, objectCoords);
    }

    void clearObjects() override
    {
        delete rtree;
        rtree = nullptr;
    }

    void runQuery(Graph &graph, unsigned int k, NodeID queryNodeID, std::vector<NodeID> &kNNs,
                  std::vector<EdgeWeight> &kNNDistances) override
    {
        queries++;
//        ier.getKNNsByALT(alt, *rtree, k, queryNodeID, kNNs, kNNDistances, graph);
        ier.getKNNsByALTOL(alt, k, queryNodeID, kNNs, kNNDistances, graph);
    }

    std::string getName() override
    {
        return "IER-ALT";
    }

    void printSummary() override {
        std::cout << "Avg candidates: " << ier.numCandidates / (float) queries << std::endl;
        std::cout << "Avg distance: " << ier.distanceSum / (double) ier.numCandidates << std::endl;
        std::cout << "Edges accessed: " << alt.edgesAccessedCount << std::endl;
    }

    void printInfo() override
    {
        std::string landmarkTypeStr;
        switch (landmarkType) {
            case LANDMARK_TYPE::RANDOM:
                landmarkTypeStr = "RANDOM";
                break;
            case LANDMARK_TYPE::RANDOM_OBJECTS:
                landmarkTypeStr = "RANDOM_OBJECTS";
                break;
            case LANDMARK_TYPE::AVOID:
                landmarkTypeStr = "AVOID";
                break;
            case LANDMARK_TYPE::AVOID_PEQUE_URATA_IRYO :
                landmarkTypeStr = "AVOID_PEQUE_URATA_IRYO";
                break;
            case LANDMARK_TYPE::ADVANCED_AVOID:
                landmarkTypeStr = "ADVANCED_AVOID";
                break;
            case LANDMARK_TYPE::FARTHEST:
                landmarkTypeStr = "FARTHEST";
                break;
            default:
                landmarkTypeStr = "UNKNOWN!";
                break;
        }
        std::cout << "landmarks: " << numLandmarks << " landmarks type: " << landmarkTypeStr << std::endl;
    }

private:
    unsigned int branchFactor;
    unsigned int numLandmarks;
    LANDMARK_TYPE landmarkType;
    ALT alt;
    IER ier;
    StaticRtree* rtree;
    int queries = 0;
};


#endif //ND_KNN_IEREXPERIMENT_H
