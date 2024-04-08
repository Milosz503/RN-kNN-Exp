//
// Created by milos on 08/03/2024.
//

#ifndef ND_KNN_ADAPTIVEEXPERIMENTSCOMMAND_H
#define ND_KNN_ADAPTIVEEXPERIMENTSCOMMAND_H


#include "../Command.h"
#include "../../processing/Graph.h"
#include "../../processing/adaptive_gtree/AdaptiveGtree.h"
#include "Experiment.h"

#include <unordered_map>

class AdaptiveExperimentsCommand : public Command {

public:
    void execute(int argc, char *argv[]) override;

    void showCommandUsage(std::string programName) override;

    void showPhaseUsage(std::string phase, std::string programName);

private:
    std::vector<NodeID> queryNodes;
    std::vector<int> kValues;
    std::size_t numSets;
    std::vector<double> objDensities;
    std::vector<std::string> objTypes;
    std::vector<int> objVariable;
    std::string filePathPrefix;
    bool verifyKNN;
    Graph graph;

    std::unordered_map<std::string, std::string> getParameters(std::string parameters);

    void
    runSingleMethodQueries(std::string method, std::string bgrFileName, std::string queryNodeFile, std::string kValues,
                           std::string parameters, std::size_t numSets, std::string objDensities, std::string objTypes,
                           std::string objVariable,
                           std::string filePathPrefix, std::string statsOutputFile);

    void runINEQueries();

    void runExperiment(Experiment &experiment);

    void
    runGtreeQueries(int fanout, int maxLeafSize);

    void
    runAGtreeQueries(int fanout, int maxLeafSize);

    void runIERQueries(unsigned int branchFactor);
    void runIERAStarQueries(unsigned int branchFactor);
    void runIERALTQueries(unsigned int branchFactor, unsigned int numLandmarks);
};


#endif //ND_KNN_ADAPTIVEEXPERIMENTSCOMMAND_H
