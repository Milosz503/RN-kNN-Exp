//
// Created by milos on 07/05/2024.
//

#ifndef ND_KNN_DISTANCEMETHOD_H
#define ND_KNN_DISTANCEMETHOD_H

#include <utility>
#include <vector>
#include "../Command.h"
#include "../../common.h"
#include "../../processing/Graph.h"
#include "../../processing/ALT.h"
#include "../../processing/AStarSearch.h"
#include "../../processing/DijkstraSearch.h"
#include "../../processing/adaptive_alt/AdaptiveALT.h"
#include "../../processing/adaptive_gtree/AdaptiveGtree.h"
#include "../../processing/pruned_highway_labeling.h"

class Query {
public:
    NodeID source;
    std::vector <NodeID> targets;
};


class DistanceMethod {
public:
    explicit DistanceMethod()
    {}

    virtual void buildIndex(Graph &graph) = 0;

    virtual void findDistances(Graph &graph, Query &query, std::vector <EdgeWeight> &distances) = 0;

    virtual void printInfo()
    {};

    virtual void printStatistics()
    {};

    virtual std::string getInfo() = 0;

    virtual std::vector<NodeID> getLandmarkNodeIDs() = 0;

    virtual ~DistanceMethod()
    {};

    virtual std::string getName() = 0;
};

class DijkstraMethod : public DistanceMethod {
public:
    DijkstraMethod()
    {}

    void buildIndex(Graph &graph) override
    {}

    void findDistances(Graph &graph, Query &query, std::vector <EdgeWeight> &distances) override
    {
        for (unsigned i = 0; i < query.targets.size(); ++i) {
            auto target = query.targets[i];
            distances[i] = dijkstra.findShortestPathDistance(graph, query.source, target);
        }
    }

    std::vector<NodeID> getLandmarkNodeIDs() override {
        return std::vector<NodeID>();
    }

    std::string getInfo() override {
        return getName();
    }

    std::string getName() override {
        return "Dijkstra";
    }


private:
    DijkstraSearch dijkstra;
};

class AStarMethod : public DistanceMethod {
public:
    AStarMethod()
    {}

    void buildIndex(Graph &graph) override
    {}

    void findDistances(Graph &graph, Query &query, std::vector <EdgeWeight> &distances) override
    {
        for (unsigned i = 0; i < query.targets.size(); ++i) {
            auto target = query.targets[i];
            distances[i] = astar.findShortestPathDistanceT(graph, query.source, target);
        }
    }

    std::vector<NodeID> getLandmarkNodeIDs() override {
        return std::vector<NodeID>();
    }

    std::string getInfo() {
        return getName();
    }

    std::string getName() override {
        return "A*";
    }

private:
    AStarSearch astar;
};

class ALTMethod : public DistanceMethod {
public:
    ALTMethod(unsigned numLandmarks, LANDMARK_TYPE landmarkType, ALTParameters parameters) :
            alt("-", 0, 0, parameters),
            numLandmarks(numLandmarks),
            landmarkType(landmarkType),
            parameters(parameters)
    {}

    void buildIndex(Graph &graph) override
    {
        alt.buildALT(graph, landmarkType, numLandmarks);
    }

    void findDistances(Graph &graph, Query &query, std::vector <EdgeWeight> &distances) override
    {
        for (unsigned i = 0; i < query.targets.size(); ++i) {
            auto target = query.targets[i];
            distances[i] = alt.findShortestPathDistance(graph, query.source, target);
        }
    }

    void printInfo() override
    {
        std::cout << "threshold: " << parameters.threshold << ", landmarks: " << alt.getLandmarksNumber() << "/" << numLandmarks << std::endl;
    }

    void printStatistics() override
    {
    }

    std::vector<NodeID> getLandmarkNodeIDs() override {
        return alt.getLandmarks();
    }

    std::string getInfo() {
        std::string landmarkTypeStr;
        switch (landmarkType) {
            case LANDMARK_TYPE::FARTHEST:
                landmarkTypeStr = "Far";
                break;
            case LANDMARK_TYPE::AVOID:
                landmarkTypeStr = "Avoid";
                break;
            case LANDMARK_TYPE::MIN_DIST:
                landmarkTypeStr = "DistS";
                break;
            case LANDMARK_TYPE::HOPS:
                landmarkTypeStr = "HopsS";
                break;
            default:
                landmarkTypeStr = "Type" + std::to_string(landmarkType);
        }
        return getName() + "_" + landmarkTypeStr + "_thr_" + std::to_string(parameters.threshold) + "_lan_" + std::to_string(numLandmarks);
    }

    std::string getName() override {
        return "ALT";
    }

private:
    ALTParameters parameters;
    unsigned numLandmarks;
    ALT alt;
    LANDMARK_TYPE landmarkType;
};

class AdaptiveALTMethod : public DistanceMethod {
public:
    explicit AdaptiveALTMethod(AdaptiveALTParams params) :
            alt(nullptr), params(params)
    {}

    void buildIndex(Graph &graph) override
    {
        alt = new AdaptiveALT(graph.getNumNodes(), graph.getNumNodes(), params);
    }

    void refineIndex(Graph &graph, Query &query)
    {
        for (unsigned i = 0; i < query.targets.size(); ++i) {
            auto target = query.targets[i];
            alt->refineIndex(graph, query.source, target);
        }
    }

    void findDistances(Graph &graph, Query &query, std::vector <EdgeWeight> &distances) override
    {
        for (unsigned i = 0; i < query.targets.size(); ++i) {
            auto target = query.targets[i];
            distances[i] = alt->findShortestPathDistance(graph, query.source, target);
        }
    }

    void printInfo() override
    {
        alt->printInfo();
    }

    void printStatistics() override
    {
        alt->printStatistics();
    }

    ~AdaptiveALTMethod() override
    {
        delete alt;
    }

    std::vector<NodeID> getLandmarkNodeIDs() override {
        return alt->getLandmarks();
    }


    std::string getInfo() override {
        std::string heuristicTypeStr;
        if(params.a == 1.0 && params.b == 0.0 && params.c == 0.0) {
            heuristicTypeStr = "DistS";
        } else if(params.a == 0.0 && params.b == 1.0 && params.c == 0.0) {
            heuristicTypeStr = "HopsS";
        } else if(params.a == 0.0 && params.b == 0.0 && params.c == 1.0) {
            heuristicTypeStr = "EstQ";
        }else {
            heuristicTypeStr = "Mixed";
        }
        return getName() + "_" + heuristicTypeStr + "_thr_" + std::to_string(params.thresholdFunction(0)) + "_lan_" + std::to_string(params.maxLandmarks) +
         "_abc_" + std::to_string(params.a) + "_" + std::to_string(params.b) + "_" + std::to_string(params.c) +"_thrT_" + params.threshold;
    }

    std::string getName() {
        return "Adaptive ALT";
    }

private:
    AdaptiveALTParams params;
    AdaptiveALT *alt;
};

class AdaptiveGtreeMethod : public DistanceMethod {
public:
    explicit AdaptiveGtreeMethod(int fanout, std::size_t maxLeafSize) :
            agtree(nullptr), fanout(fanout), maxLeafSize(maxLeafSize)
    {}

    void buildIndex(Graph &graph) override
    {
        agtree = new AdaptiveGtree(graph.getNetworkName(), graph.getNumNodes(),
                                   graph.getNumEdges(), fanout, maxLeafSize);
        agtree->buildGtree(graph);
    }

    void findDistances(Graph &graph, Query &query, std::vector <EdgeWeight> &distances) override
    {
        for (unsigned i = 0; i < query.targets.size(); ++i) {
            auto target = query.targets[i];
            distances[i] = agtree->getShortestPathDistance(graph, query.source, target);
        }
    }

    void printInfo() override
    {
        std::cout << "fanout: " << fanout << ", maxLeafSize: " << maxLeafSize << ", levels: " << agtree->getNumLevels()
                  << std::endl;
    }

    void printStatistics() override
    {

    }

    ~AdaptiveGtreeMethod() override
    {
        delete agtree;
    }

    std::vector<NodeID> getLandmarkNodeIDs() override {
        return std::vector<NodeID>();
    }

    std::string getInfo() {
        return getName() + "_f_" + std::to_string(fanout) + "_t_" + std::to_string(maxLeafSize) + "_l_" + std::to_string(agtree->getNumLevels());
    }

    std::string getName() {
        return "Adaptive Gtree";
    }

private:
    int fanout;
    std::size_t maxLeafSize;
    AdaptiveGtree *agtree;
};


class GtreeMethod : public DistanceMethod {
public:
    explicit GtreeMethod(int fanout, std::size_t maxLeafSize) :
            gtree(nullptr), fanout(fanout), maxLeafSize(maxLeafSize)
    {}

    void buildIndex(Graph &graph) override
    {
        gtree = new Gtree(graph.getNetworkName(), graph.getNumNodes(),
                          graph.getNumEdges(), fanout, maxLeafSize);
        gtree->buildGtree(graph);
    }

    void findDistances(Graph &graph, Query &query, std::vector <EdgeWeight> &distances) override
    {
        for (unsigned i = 0; i < query.targets.size(); ++i) {
            auto target = query.targets[i];
            distances[i] = gtree->getShortestPathDistance(graph, query.source, target);
        }
    }

    void printInfo() override
    {
        std::cout << "fanout: " << fanout << ", maxLeafSize: " << maxLeafSize << ", levels: " << gtree->getNumLevels()
                  << std::endl;
    }

    void printStatistics() override
    {

    }

    ~GtreeMethod() override
    {
        delete gtree;
    }

    std::vector<NodeID> getLandmarkNodeIDs() override {
        return std::vector<NodeID>();
    }

    std::string getInfo() {
        return getName() + "_f_" + std::to_string(fanout) + "_t_" + std::to_string(maxLeafSize) + "_l_" + std::to_string(gtree->getNumLevels());
    }

    std::string getName() {
        return "Gtree";
    }

private:
    int fanout;
    std::size_t maxLeafSize;
    Gtree *gtree;
};

class PhlMethod : public DistanceMethod {
public:
    explicit PhlMethod()
    {}

    void buildIndex(Graph &graph) override
    {
        std::cout << "PhlMethod::buildIndex not implemented!" << std::endl;
        exit(1);
    }

    double buildIndex(Graph& graph, std::string tsvPath)
    {
        std::cout << "Writing TSV file..." << std::endl;
        graph.outputToTSVFile(tsvPath);
        std::cout << "TSV file written successfully." << std::endl;
        phl.ConstructLabel(tsvPath.c_str());
        return phl.getConstructionTime();
    }

    void findDistances(Graph &graph, Query &query, std::vector <EdgeWeight> &distances) override
    {
        for (unsigned i = 0; i < query.targets.size(); ++i) {
            auto target = query.targets[i];
            distances[i] = phl.Query(query.source, target);
        }
    }

    void printInfo() override
    {
        phl.Statistics();
    }

    void printStatistics() override
    {

    }

    ~PhlMethod() override
    {

    }

    std::vector<NodeID> getLandmarkNodeIDs() override {
        return std::vector<NodeID>();
    }

    std::string getInfo() {
        return getName();
    }

    std::string getName() {
        return "PHL";
    }

private:
    int fanout;
    std::size_t maxLeafSize;
    PrunedHighwayLabeling phl;
};


#endif //ND_KNN_DISTANCEMETHOD_H
