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
#include "../../processing/adaptive_gtree/AdaptiveGtree.h"
#include "DistanceMethod.h"


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

    void buildIndexes(std::vector<std::vector<double>>& results);

    void loadQueries();

    void runAll();

    void runAll(std::vector<std::vector<double>>& results);

    void runMethod(DistanceMethod* method);

    void runMethod(DistanceMethod* method, std::vector<std::vector<double>>& results, int iter);

    void validateAll();

    void compareDecayFunctions(int numRepeats, const std::string network);

    void createMethodsConstABestThreshold(int numRepeats, const std::string network, int numQuerySteps);

    void createMethodsConstBBestThreshold(int numRepeats, const std::string network, int numQuerySteps);

    void createMethodsConstCBestThreshold(int numRepeats, const std::string network, int numQuerySteps);

    void compareDecayFunctions(int numRepeats, const std::string network, int numQuerySteps);

    void compareThresholdLandmarkAdaptive(int numRepeats, const std::string network, int numQuerySteps);

    void compareThresholdLandmark(int numRepeats, const std::string network, LANDMARK_TYPE landmarkType, int numQuerySteps);

    void compareMethods(int numRepeats, const std::string network, int numQuerySteps);

    void clearMethods();

//    void runMultiTargetALT();
};


#endif //ND_KNN_DISTANCEEXPERIMENTCOMMAND_H
