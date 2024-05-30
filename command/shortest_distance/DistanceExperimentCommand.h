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
    std::string network = "";
    bool validate = true;
    unsigned numRepeats = 1;
    std::string resultsPath = "./";
    std::vector<std::vector<double>> results;

    std::vector<DistanceMethod *> methods;

    void buildIndexes(bool saveTimes = true);

    void loadQueries();

    void runAll(bool saveOnlyLastResult = false);

    void runMethod(DistanceMethod* method);

    void runMethod(DistanceMethod* method, int iter, bool saveOnlyLastResult = false);

    void validateAll();

    void compareLandmarksNumber();

    void compareFarthestALT();

    void compareAvoidALT();

    void compareNumLandmarksMinDistALT();

    void compareThresholdMinDistALT();

    void compareNumLandmarksHopsALT();

    void compareThresholdHopsALT();


    void createMethodsConstABestThreshold();

    void createMethodsConstBBestThreshold();

    void createMethodsConstCBestThreshold();

    void compareDecayFunctions();

    void compareThresholdLandmarkAdaptive(int numRepeats);

    void compareThresholdLandmark(int numRepeats, LANDMARK_TYPE landmarkType);

    void runStandardTestCase(const std::function<void()>& testCase);

    void compareMethods();

    void clearMethods();

    void visualizeQueries(std::string name);

    void exportLandmarks(std::string name);

//    void runMultiTargetALT();
};


#endif //ND_KNN_DISTANCEEXPERIMENTCOMMAND_H
