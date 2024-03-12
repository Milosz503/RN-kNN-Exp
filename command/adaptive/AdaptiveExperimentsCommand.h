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
    std::unordered_map<std::string, std::string> getParameters(std::string parameters);

    void
    runSingleMethodQueries(std::string method, std::string bgrFileName, std::string queryNodeFile, std::string kValues,
                           std::string parameters, std::size_t numSets, std::string objDensities, std::string objTypes,
                           std::string objVariable,
                           std::string filePathPrefix, std::string statsOutputFile);

    void runQueries(std::string bgrFileName, std::string queryNodeFile, std::string kValues, std::string parameters,
                    std::size_t numSets,
                    std::string objDensities, std::string objTypes, std::string objVariable, std::string filePathPrefix,
                    std::string statsOutputFile);

    void runINEQueries(Graph &graph, std::vector<NodeID> &queryNodes, std::vector<int> &kValues, std::size_t numSets,
                       std::vector<double> objDensities, std::vector<std::string> objTypes,
                       std::vector<int> objVariable, std::string filePathPrefix,
                       std::string statsOutputFile, std::vector<std::string> specialFields = {});

    void runExperiment(Experiment &experiment, Graph &graph, std::vector<NodeID> &queryNodes, std::vector<int> &kValues,
                       std::size_t numSets, std::vector<double> objDensities, std::vector<std::string> objTypes,
                       std::vector<int> objVariable, std::string filePathPrefix, std::string statsOutputFile,
                       bool verifyKNN, std::vector<std::string> &parameterKeys,
                       std::vector<std::string> &parameterValues, std::vector<std::string> specialFields = {});

// TODO: Implement INE queries by dynamic graph
//    void runINEQueriesByDynamicGraph(Graph &graph, std::string dynBgrFileName, std::vector<NodeID> &queryNodes,
//                                     std::vector<int> &kValues, std::size_t numSets,
//                                     std::vector<double> objDensities, std::vector<std::string> objTypes,
//                                     std::vector<int> objVariable, std::string filePathPrefix,
//                                     std::string statsOutputFile, bool verifyKNN,
//                                     std::vector<std::string> specialFields = {});

    void
    runGtreeQueries(Graph &graph, std::string gtreeIdxFile, std::vector<NodeID> &queryNodes, std::vector<int> &kValues,
                    std::size_t numSets, std::vector<double> objDensities, std::vector<std::string> objTypes,
                    std::vector<int> objVariable, std::string filePathPrefix,
                    std::string statsOutputFile, bool verifyKNN, std::vector<std::string> &parameterKeys,
                    std::vector<std::string> &parameterValues,
                    std::vector<std::string> specialFields = {});

    void
    runAGtreeQueries(Graph &graph, std::string gtreeIdxFile, std::vector<NodeID> &queryNodes, std::vector<int> &kValues,
                     std::size_t numSets, std::vector<double> objDensities, std::vector<std::string> objTypes,
                     std::vector<int> objVariable, std::string filePathPrefix,
                     std::string statsOutputFile, bool verifyKNN, std::vector<std::string> &parameterKeys,
                     std::vector<std::string> &parameterValues,
                     std::vector<std::string> specialFields = {});
};


#endif //ND_KNN_ADAPTIVEEXPERIMENTSCOMMAND_H
