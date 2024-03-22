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

class Experiment {
public:
    virtual void buildIndex(Graph &graph) = 0;

    virtual void loadObjects(Graph &graph, std::string filePathPrefix,
                             std::string setType, double setDensity, int setVariable, int setIdx,
                             std::vector<std::string> &parameterNames, std::vector<std::string> &parameterValues) = 0;

    virtual void runQuery(Graph &graph, unsigned int k, NodeID queryNodeID, std::vector<NodeID> &kNNs,
                          std::vector<EdgeWeight> &kNNDistances) = 0;

    virtual std::string getName() = 0;

    virtual void printInfo()
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
    loadObjects(Graph &graph, std::string filePathPrefix,
                std::string setType, double setDensity,
                int setVariable, int setIdx, std::vector<std::string> &parameterNames,
                std::vector<std::string> &parameterValues) override
    {
        int setSize;
        std::string objSetFile = filePathPrefix + "/obj_indexes/" +
                                 utility::constructObjsectSetFileName(graph.getNetworkName(), setType, setDensity,
                                                                      setVariable, setIdx);
        std::vector<NodeID> objectNodes = utility::getPointSetFromFile(objSetFile, setType, setDensity, setSize,
                                                                       setVariable);

        graph.resetAllObjects();
        graph.parseObjectSet(objectNodes);
    }

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
    loadObjects(Graph &graph, std::string filePathPrefix,
                std::string setType, double setDensity,
                int setVariable, int setIdx, std::vector<std::string> &parameterNames,
                std::vector<std::string> &parameterValues) override
    {
        std::string objIdxFilePath = filePathPrefix + "/obj_indexes/" +
                                     utility::constructObjectIndexFileName(gtree->getNetworkName(),
                                                                           constants::OBJ_IDX_GTREE, setType,
                                                                           setDensity, setVariable, setIdx,
                                                                           parameterNames, parameterValues);
        delete occList;
        int setSize;
        std::string objSetFile = filePathPrefix + "/obj_indexes/" +
                                 utility::constructObjsectSetFileName(gtree->getNetworkName(), setType, setDensity,
                                                                      setVariable, setIdx);
        std::vector<NodeID> objectNodes = utility::getPointSetFromFile(objSetFile, setType, setDensity, setSize,
                                                                       setVariable);
        //assert(objTypes[i] == setType && objDensities[j] == setDensity);

        occList = new OccurenceList(setType, setDensity, setVariable, setSize);
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
    loadObjects(Graph &graph, std::string filePathPrefix,
                std::string setType, double setDensity,
                int setVariable, int setIdx, std::vector<std::string> &parameterNames,
                std::vector<std::string> &parameterValues) override
    {
        std::string objIdxFilePath = filePathPrefix + "/obj_indexes/" +
                                     utility::constructObjectIndexFileName(agtree->getNetworkName(),
                                                                           constants::OBJ_IDX_GTREE, setType,
                                                                           setDensity, setVariable, setIdx,
                                                                           parameterNames, parameterValues);
        delete occList;
        int setSize;
        std::string objSetFile = filePathPrefix + "/obj_indexes/" +
                                 utility::constructObjsectSetFileName(agtree->getNetworkName(), setType, setDensity,
                                                                      setVariable, setIdx);
        std::vector<NodeID> objectNodes = utility::getPointSetFromFile(objSetFile, setType, setDensity, setSize,
                                                                       setVariable);
        //assert(objTypes[i] == setType && objDensities[j] == setDensity);

        occList = new OccurenceList(setType, setDensity, setVariable, setSize);
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

    IERExperiment() :
            rtree(nullptr)
    {}

    void buildIndex(Graph &graph) override
    {

    }

    void
    loadObjects(Graph &graph, std::string filePathPrefix,
                std::string setType, double setDensity,
                int setVariable, int setIdx, std::vector<std::string> &parameterNames,
                std::vector<std::string> &parameterValues) override
    {
        std::string objIdxFilePath = filePathPrefix + "/obj_indexes/" + utility::constructObjectIndexFileName(graph.getNetworkName(),constants::OBJ_IDX_RTREE,setType,setDensity,setVariable,setIdx,parameterNames,parameterValues);
        delete rtree;
        rtree = serialization::getIndexFromBinaryFileDynamic<StaticRtree>(objIdxFilePath);

    }

    void runQuery(Graph &graph, unsigned int k, NodeID queryNodeID, std::vector<NodeID> &kNNs,
                  std::vector<EdgeWeight> &kNNDistances) override
    {
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

private:
    IER ier;
    StaticRtree* rtree;
};

#endif //ND_KNN_EXPERIMENT_H
