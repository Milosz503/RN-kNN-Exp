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

class Experiment {
public:
    virtual void buildIndex(Graph &graph) = 0;

    virtual void loadObjects(Graph &graph, std::string filePathPrefix,
                             std::string setType, double setDensity, int setVariable, int setIdx,
                             std::vector<std::string> &parameterNames, std::vector<std::string> &parameterValues) = 0;

    virtual void runQuery(Graph &graph, unsigned int k, NodeID queryNodeID, std::vector<NodeID> &kNNs,
                          std::vector<EdgeWeight> &kNNDistances) = 0;

    virtual std::string getName() = 0;

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
        occList = new OccurenceList(serialization::getIndexFromBinaryFile<OccurenceList>(objIdxFilePath));

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

    ~GTreeExperiment() override
    {
        delete gtree;
    }

private:
    int fanout;
    std::size_t maxLeafSize;
    Gtree *gtree;
    OccurenceList* occList;
};

class AdaptiveGTreeExperiment : public Experiment {
public:

    AdaptiveGTreeExperiment(int fanout, std::size_t maxLeafSize) :
            agtree(nullptr), fanout(fanout), maxLeafSize(maxLeafSize)
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
        occList = serialization::getIndexFromBinaryFile<OccurenceList>(objIdxFilePath);

    }

    void runQuery(Graph &graph, unsigned int k, NodeID queryNodeID, std::vector<NodeID> &kNNs,
                  std::vector<EdgeWeight> &kNNDistances) override
    {
        agtree->getKNNs(occList, k, queryNodeID, kNNs, kNNDistances, graph);
    }

    std::string getName() override
    {
        return "AdaptiveGTree";
    }

    ~AdaptiveGTreeExperiment() override
    {
        delete agtree;
    }

private:
    int fanout;
    std::size_t maxLeafSize;
    AdaptiveGtree *agtree;
    OccurenceList occList;
};

#endif //ND_KNN_EXPERIMENT_H
