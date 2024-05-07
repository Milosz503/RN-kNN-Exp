//
// Created by milos on 21/03/2024.
//

#ifndef ND_KNN_DISTANCEEXPERIMENTCOMMAND_H
#define ND_KNN_DISTANCEEXPERIMENTCOMMAND_H


#include <utility>
#include <vector>
#include "../Command.h"
#include "../../common.h"
#include "../../processing/Graph.h"
#include "../../processing/ALT.h"
#include "../../processing/AStarSearch.h"
#include "../../processing/DijkstraSearch.h"
#include "../../processing/adaptive_alt/AdaptiveALT.h"

class Query {
public:
    NodeID source;
    std::vector<NodeID> targets;
};


class DistanceMethod {
public:
    explicit DistanceMethod(std::string name) : name(std::move(name))
    {}

    virtual void buildIndex(Graph &graph) = 0;

    virtual void findDistances(Graph &graph, Query &query, std::vector<EdgeWeight> &distances) = 0;
    virtual void printInfo() {};
    virtual void printStatistics() {};

    virtual ~DistanceMethod() {};

    const std::string name;
};

class DijkstraMethod : public DistanceMethod {
public:
    DijkstraMethod() : DistanceMethod("Dijkstra") {}

    void buildIndex(Graph &graph) override
    {}

    void findDistances(Graph &graph, Query &query, std::vector<EdgeWeight> &distances) override
    {
        for (unsigned i = 0; i < query.targets.size(); ++i) {
            auto target = query.targets[i];
            distances[i] = dijkstra.findShortestPathDistance(graph, query.source, target);
        }
    }

private:
    DijkstraSearch dijkstra;
};

class AStarMethod : public DistanceMethod {
public:
    AStarMethod(): DistanceMethod("AStar") {}

    void buildIndex(Graph &graph) override
    {}

    void findDistances(Graph &graph, Query &query, std::vector<EdgeWeight> &distances) override
    {
        for (unsigned i = 0; i < query.targets.size(); ++i) {
            auto target = query.targets[i];
            distances[i] = astar.findShortestPathDistanceT(graph, query.source, target);
        }
    }

private:
    AStarSearch astar;
};

class ALTMethod : public DistanceMethod {
public:
    ALTMethod(unsigned numLandmarks, LANDMARK_TYPE landmarkType, ALTParameters parameters) :
            DistanceMethod("ALT"),
            alt("-", 0, 0, parameters),
            numLandmarks(numLandmarks),
            landmarkType(landmarkType),
            parameters(parameters)
    {}

    void buildIndex(Graph &graph) override
    {
        alt.buildALT(graph, landmarkType, numLandmarks);
    }

    void findDistances(Graph &graph, Query &query, std::vector<EdgeWeight> &distances) override
    {
        for (unsigned i = 0; i < query.targets.size(); ++i) {
            auto target = query.targets[i];
            distances[i] = alt.findShortestPathDistance(graph, query.source, target);
        }
    }

    void printInfo() override
    {
        std::cout << "threshold: " << parameters.threshold << std::endl;
    }

    void printStatistics() override
    {
        std::cout << "Number of landmarks: " << numLandmarks << std::endl;
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
            DistanceMethod("Adaptive ALT"), alt(nullptr), params(params)
    {}

    void buildIndex(Graph &graph) override
    {
        alt = new AdaptiveALT(graph.getNumNodes(), graph.getNumNodes(), params);
    }

    void findDistances(Graph &graph, Query &query, std::vector<EdgeWeight> &distances) override
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

    ~AdaptiveALTMethod() override {
        delete alt;
    }

private:
    AdaptiveALTParams params;
    AdaptiveALT* alt;
};


class DistanceExperimentCommand : public Command {

public:
    void execute(int argc, char *argv[]) override;

    void showCommandUsage(std::string programName) override;

    ~DistanceExperimentCommand() override;

private:
    Graph graph;
    std::vector<Query> queries;
    unsigned long numQueries;
    unsigned long maxDist;
    unsigned long numTargets;
    unsigned long numLandmarks;

    std::vector<DistanceMethod *> methods;



    void buildIndexes();

    void loadQueries();

    void runAll();

    void runMethod(DistanceMethod* method);

    void validateAll();

//    void runMultiTargetALT();
};


#endif //ND_KNN_DISTANCEEXPERIMENTCOMMAND_H
