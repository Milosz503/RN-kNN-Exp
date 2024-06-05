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

enum class WorkloadType {
    RANDOM = 0,
    CLUSTER = 1,
};

enum class ResultsType {
    POWER_OF_2 = 0,
    LAST = 1,
    NONE = 2,
};

class DistanceExperimentCommand : public Command {

public:
    void execute(int argc, char *argv[]) override;

    void showCommandUsage(std::string programName) override;

    ~DistanceExperimentCommand() override;

private:
    std::string graphPath;
    std::string dataPath;
    std::string tsvPath;
    Graph graph;
    std::vector<Query> queries;
    unsigned long numQueries;
    unsigned long maxDist;
    unsigned long numTargets;
    std::string network = "";
    WorkloadType workloadType = WorkloadType::RANDOM;
    bool validate = true;
    unsigned numRepeats = 1;
    std::string resultsPath = "./";
    std::vector<std::vector<double>> results;

    std::vector<DistanceMethod *> methods;

    void buildIndexes(bool saveTimes = true);

    void loadQueries();

    void runAll(ResultsType resultsType = ResultsType::POWER_OF_2);

    void runMethod(DistanceMethod* method);

    void runMethod(DistanceMethod* method, int iter, ResultsType resultsType = ResultsType::POWER_OF_2);

    void validateAll();

    void compareOtherMethods();

    void compareAdaptiveEstThresholdQueryVsTime();

    void compareAdaptiveDistThresholdQueryVsTime();

    void compareAdaptiveHopsThresholdQueryVsTime();

    void compareAltEstLandmarksVsThreshold();

    void compareAltHopsLandmarksVsThreshold();

    void compareAltDistLandmarksVsThreshold();

    void visualizeQueries();

    void visualizeLandmarks();

    void compareLandmarksNumberVsTime();

    void compareFarthestALT();

    void compareAvoidALT();

    void compareAltEstThresholdQueryVsLandmarks();

    void compareAltDistThresholdQueryVsLandmarks();

    void compareAltDistThresholdQueryVsTime();

    void compareAltHopsThresholdQueryVsLandmarks();

    void compareAltHopsThresholdQueryVsTime();


    void createMethodsConstABestThreshold();

    void createMethodsConstBBestThreshold();

    void createMethodsConstCBestThreshold();

    void compareDecayFunctions();

    void compareThresholdLandmarkAdaptive(int numRepeats);

    void compareThresholdLandmark(int numRepeats, LANDMARK_TYPE landmarkType);

    void runStandardTestCase(const std::function<void()>& testCase);

    void compareMethods();

    void clearMethods();

    void saveQueries(std::string name);

    void exportLandmarks(std::string name);

    std::string getResultsPath();

//    void runMultiTargetALT();
};


#endif //ND_KNN_DISTANCEEXPERIMENTCOMMAND_H
