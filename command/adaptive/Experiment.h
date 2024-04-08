//
// Created by milos on 08/03/2024.
//

#ifndef ND_KNN_EXPERIMENT_H
#define ND_KNN_EXPERIMENT_H

#include "../../processing/DynamicGraph.h"
#include "../../processing/Graph.h"
#include "../../common.h"
#include "../../utility/Statistics.h"
#include "../../processing/INE.h"
#include "../../utility/StopWatch.h"
#include "../../processing/Gtree.h"
#include "../../processing/adaptive_gtree/AdaptiveGtree.h"
#include "../../utility/utility.h"
#include "../../utility/serialization.h"
#include "../../processing/IER.h"
#include "../../processing/ALT.h"

class Experiment {
public:
    virtual void buildIndex(Graph &graph) = 0;

    virtual void loadObjects(Graph &graph, std::vector<NodeID>& objectNodes) = 0;
    virtual void clearObjects() = 0;

    virtual void runQuery(Graph &graph, unsigned int k, NodeID queryNodeID, std::vector<NodeID> &kNNs,
                          std::vector<EdgeWeight> &kNNDistances) = 0;

    virtual std::string getName() = 0;

    virtual void printInfo()
    {}

    virtual void printSummary()
    {}

    virtual ~Experiment() = default;
};

class INEExperiment : public Experiment {
public:
    INEExperiment()
    {

    }

    void buildIndex(Graph &graph) override
    {
        // Do nothing
    }

    void
    loadObjects(Graph &graph, std::vector<NodeID>& objectNodes) override {}

    void clearObjects() override {}

    void runQuery(Graph &graph, unsigned int k, NodeID queryNodeID, std::vector<NodeID> &kNNs,
                  std::vector<EdgeWeight> &kNNDistances) override
    {
        ine.getKNNs(graph, k, queryNodeID, kNNs, kNNDistances);
    }

    std::string getName() override
    {
        return "INE";
    }

private:
    INE ine;
};

class GTreeExperiment : public Experiment {
public:

    GTreeExperiment(int fanout, std::size_t maxLeafSize) :
            gtree(nullptr), occList(nullptr), fanout(fanout), maxLeafSize(maxLeafSize)
    {}

    GTreeExperiment(const GTreeExperiment &other)
    {
        assert(false);
    }

    GTreeExperiment &operator=(const GTreeExperiment &other)
    {
        assert(false);
    }

    void buildIndex(Graph &graph) override
    {
        gtree = new Gtree(graph.getNetworkName(), graph.getNumNodes(),
                          graph.getNumEdges(), fanout, maxLeafSize);
        gtree->buildGtree(graph);
    }

    void
    loadObjects(Graph &graph, std::vector<NodeID>& objectNodes) override
    {
        //assert(objTypes[i] == setType && objDensities[j] == setDensity);
        occList = new OccurenceList();
        for (auto objIt = objectNodes.begin(); objIt != objectNodes.end(); ++objIt) {
            // Find the Gtree leaf index for this object and add it to occurence list
            // for that leaf (create list if it doesn't exist)
            int leafIdx = gtree->getLeafIndex(*objIt);
            occList->addLeafOccurence(leafIdx, *objIt);

            // Propagate this to occurence lists of parents of leaf node
            int childIdx = leafIdx;
            int parentIdx = gtree->getParentIndex(leafIdx);
            while (parentIdx != -1) {
                occList->addParentOccurence(parentIdx, childIdx);

                // Go up a level (until we reach root)
                childIdx = parentIdx;
                parentIdx = gtree->getParentIndex(childIdx);
            }
        }

    }

    void clearObjects() override
    {
        delete occList;
        occList = nullptr;
    }

    void runQuery(Graph &graph, unsigned int k, NodeID queryNodeID, std::vector<NodeID> &kNNs,
                  std::vector<EdgeWeight> &kNNDistances) override
    {
        gtree->getKNNs(*occList, k, queryNodeID, kNNs, kNNDistances, graph);
    }

    std::string getName() override
    {
        return "GTree";
    }

    void printInfo() override
    {
        std::cout << "fanout: " << fanout << ", maxLeafSize: " << maxLeafSize << std::endl;
    }

    ~GTreeExperiment() override
    {
        delete gtree;
    }

private:
    int fanout;
    std::size_t maxLeafSize;
    Gtree *gtree;
    OccurenceList *occList;
};

class AdaptiveGTreeExperiment : public Experiment {
public:

    AdaptiveGTreeExperiment(int fanout, std::size_t maxLeafSize) :
            occList(nullptr), agtree(nullptr), fanout(fanout), maxLeafSize(maxLeafSize)
    {}

    void buildIndex(Graph &graph) override
    {
        agtree = new AdaptiveGtree(graph.getNetworkName(), graph.getNumNodes(),
                                   graph.getNumEdges(), fanout, maxLeafSize);
        agtree->buildGtree(graph);
    }

    void
    loadObjects(Graph &graph, std::vector<NodeID>& objectNodes) override
    {
        occList = new OccurenceList();
        for (auto objIt = objectNodes.begin(); objIt != objectNodes.end(); ++objIt) {
            // Find the Gtree leaf index for this object and add it to occurence list
            // for that leaf (create list if it doesn't exist)
            int leafIdx = agtree->getLeafIndex(*objIt);
            occList->addLeafOccurence(leafIdx, *objIt);

            // Propagate this to occurence lists of parents of leaf node
            int childIdx = leafIdx;
            int parentIdx = agtree->getParentIndex(leafIdx);
            while (parentIdx != -1) {
                occList->addParentOccurence(parentIdx, childIdx);

                // Go up a level (until we reach root)
                childIdx = parentIdx;
                parentIdx = agtree->getParentIndex(childIdx);
            }
        }
    }

    void clearObjects() override
    {
        delete occList;
        occList = nullptr;
    }

    void runQuery(Graph &graph, unsigned int k, NodeID queryNodeID, std::vector<NodeID> &kNNs,
                  std::vector<EdgeWeight> &kNNDistances) override
    {
        agtree->getKNNs(*occList, k, queryNodeID, kNNs, kNNDistances, graph);
    }

    std::string getName() override
    {
        return "AdaptiveGTree";
    }

    void printInfo() override
    {
        std::cout << "fanout: " << fanout << ", maxLeafSize: " << maxLeafSize << ", levels: " << agtree->getNumLevels()
                  << std::endl;
    }

    ~AdaptiveGTreeExperiment() override
    {
        delete agtree;
    }

private:
    int fanout;
    std::size_t maxLeafSize;
    AdaptiveGtree *agtree;
    OccurenceList *occList;
};

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

    explicit IERALTExperiment(unsigned int branchFactor, unsigned int numLandmarks) :
            branchFactor(branchFactor),
            numLandmarks(numLandmarks),
            rtree(nullptr)
    {}

    void buildIndex(Graph &graph) override
    {
        alt.buildALT(graph, LANDMARK_TYPE::RANDOM, numLandmarks);
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
    unsigned int numLandmarks;
    ALT alt;
    IER ier;
    StaticRtree* rtree;
};

#endif //ND_KNN_EXPERIMENT_H
