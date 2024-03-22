//
// Created by milos on 21/03/2024.
//

#ifndef ND_KNN_DISTANCEEXPERIMENTCOMMAND_H
#define ND_KNN_DISTANCEEXPERIMENTCOMMAND_H


#include <vector>
#include "../Command.h"
#include "../../common.h"
#include "../../processing/Graph.h"

class Query {
public:
    NodeID source;
    std::vector<NodeID> targets;
};

class DistanceExperimentCommand : public Command {

public:
    void execute(int argc, char *argv[]) override;

    void showCommandUsage(std::string programName) override;

private:
    Graph graph;
    std::vector<Query> queries;
    unsigned long numQueries;
    unsigned long maxDist;
    unsigned long numTargets;

    void loadQueries();

    void runAll();

    void runMethod(std::function<void()> method, std::string methodName);

    void validateAll();

    void runDijkstra();

    void runAStar();


};


#endif //ND_KNN_DISTANCEEXPERIMENTCOMMAND_H
