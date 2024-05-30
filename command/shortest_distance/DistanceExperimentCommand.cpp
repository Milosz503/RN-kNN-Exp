//
// Created by milos on 21/03/2024.
//

#include "DistanceExperimentCommand.h"
#include "../../utility/serialization.h"
#include "../../utility/StopWatch.h"
#include "../../processing/DijkstraSearch.h"
#include "../../processing/AStarSearch.h"
#include "QueryGenerator.h"
#include "../../utils/Utils.h"
#include <fstream>

void DistanceExperimentCommand::compareLandmarksNumber()
{
    for (int i = 2; i <= 40; i += 2) {
        methods.push_back(new ALTMethod(i, LANDMARK_TYPE::FARTHEST, {}));
    }
    results.resize(methods.size());

    for (unsigned i = 0; i < numRepeats; ++i) {
        std::cout << "Repeat: " << i << std::endl;
        buildIndexes(false);
        loadQueries();
        runAll(true);
    }

    queries.clear();
}

void DistanceExperimentCommand::compareFarthestALT()
{
    std::vector<unsigned> config = {
            6, 8, 12, 20, 36
    };

    for(auto landmarksNum : config) {
        methods.push_back(new ALTMethod(
                landmarksNum,
                LANDMARK_TYPE::FARTHEST,
                ALTParameters()
        ));
    }
    results.resize(methods.size());

    for (int i = 0; i < numRepeats; i++) {
        std::cout << "Repeat: " << i << std::endl;
        buildIndexes();
        loadQueries();
        runAll();
        queries.clear();
    }
}

void DistanceExperimentCommand::compareAvoidALT()
{
    std::vector<unsigned> config = {
            6, 8, 12, 20, 36
    };

    for(auto landmarksNum : config) {
        methods.push_back(new ALTMethod(
                landmarksNum,
                LANDMARK_TYPE::AVOID,
                ALTParameters()
        ));
    }
    results.resize(methods.size());

    for (int i = 0; i < numRepeats; i++) {
        std::cout << "Repeat: " << i << std::endl;
        buildIndexes();
        loadQueries();
        runAll();
        queries.clear();
    }
}

void DistanceExperimentCommand::compareNumLandmarksMinDistALT()
{
    std::vector<double> values = {
            0.16, 0.20, 0.24, 0.28, 0.32, 0.36
    };

    results.resize(values.size());
    for (unsigned i = 0; i < values.size(); ++i) {
        methods.push_back(new ALTMethod(
                150,
                LANDMARK_TYPE::MIN_DIST,
                ALTParameters(values[i], numQueries, &results[i])
        ));
    }

    for (int i = 0; i < numRepeats; i++) {
        std::cout << "Repeat: " << i << std::endl;
        buildIndexes(false);
    }

    // Do not average results, doesn't make sense
    write_to_csv(results, methods, resultsPath + "/results", 1);
}

void DistanceExperimentCommand::compareThresholdMinDistALT()
{
    std::vector<std::tuple<int, double>> config = {
            {60, 0.16},
            {37, 0.20},
            {27, 0.24},
            {18, 0.28},
            {12, 0.32},
            {10, 0.36},
    };

    for(auto parameters : config) {
        methods.push_back(new ALTMethod(
                std::get<0>(parameters),
                LANDMARK_TYPE::MIN_DIST,
                ALTParameters(std::get<1>(parameters), numQueries)
        ));
    }
    results.resize(methods.size());

    for (int i = 0; i < numRepeats; i++) {
        std::cout << "Repeat: " << i << std::endl;
        buildIndexes();
        loadQueries();
        runAll();
        queries.clear();
    }
}

void DistanceExperimentCommand::compareNumLandmarksHopsALT()
{
    std::vector<double> values = {
            0.15, 0.20, 0.25, 0.30, 0.35, 0.40
    };
    results.resize(values.size());
    for (unsigned i = 0; i < values.size(); ++i) {
        methods.push_back(new ALTMethod(
                150,
                LANDMARK_TYPE::HOPS,
                ALTParameters(values[i], numQueries, &results[i])
        ));
    }

    for (int i = 0; i < numRepeats; i++) {
        std::cout << "Repeat: " << i << std::endl;
        buildIndexes(false);
    }

    // Do not average results, doesn't make sense
    write_to_csv(results, methods, resultsPath + "/results", 1);
}

void DistanceExperimentCommand::compareThresholdHopsALT()
{
    std::vector<std::tuple<int, double>> config = {
            {35, 0.15},
            {23, 0.20},
            {14, 0.25},
            {12, 0.30},
            {10, 0.35},
            {8, 0.40},
    };

    for(auto parameters : config) {
        methods.push_back(new ALTMethod(
                std::get<0>(parameters),
                LANDMARK_TYPE::HOPS,
                ALTParameters(std::get<1>(parameters), numQueries)
        ));
    }
    results.resize(methods.size());

    for (int i = 0; i < numRepeats; i++) {
        std::cout << "Repeat: " << i << std::endl;
        buildIndexes();
        loadQueries();
        runAll();
        queries.clear();
    }
}

void DistanceExperimentCommand::createMethodsConstABestThreshold()
{
    for (int i = 0; i < 11; i++) {
        methods.push_back(new AdaptiveALTMethod(AdaptiveALTParams(20, 0.0, i / 10.0, (10 - i) / 10.0, [](unsigned q) {
            return 0.15 * (pow(0.69, 0.0025 * q)) + 0.15;
        })));
    }
    results.resize(methods.size());

    for (int i = 0; i < numRepeats; i++) {
        std::cout << "Repeat: " << i << std::endl;
        buildIndexes();
        loadQueries();
        runAll();
        queries.clear();


        clearMethods();
        results.clear();
    }
}

void DistanceExperimentCommand::createMethodsConstBBestThreshold()
{
    for (int i = 0; i < 11; i++) {
        methods.push_back(new AdaptiveALTMethod(AdaptiveALTParams(20, i / 10.0, 0.0, (10 - i) / 10.0, [](unsigned q) {
            return 0.15 * (pow(0.69, 0.0025 * q)) + 0.15;
        })));
    }
    results.resize(methods.size());

    for (int i = 0; i < numRepeats; i++) {
        std::cout << "Repeat: " << i << std::endl;
        buildIndexes();
        loadQueries();
        runAll();
        queries.clear();
    }
}

void DistanceExperimentCommand::createMethodsConstCBestThreshold()
{
    for (int i = 0; i < 11; i++) {
        methods.push_back(new AdaptiveALTMethod(AdaptiveALTParams(20, i / 10.0, (10 - i) / 10.0, 0.0, [](unsigned q) {
            return 0.15 * (pow(0.69, 0.0025 * q)) + 0.15;
        })));
    }
    results.resize(methods.size());

    for (int i = 0; i < numRepeats; i++) {
        std::cout << "Repeat: " << i << std::endl;
        buildIndexes();
        loadQueries();
        runAll();
        queries.clear();
    }
}


void DistanceExperimentCommand::compareDecayFunctions()
{
    methods.push_back(new AdaptiveALTMethod(
            AdaptiveALTParams(20, 0.0, 1.0, 0.0, [](unsigned q) { return 0.15 * (pow(0.69, 0.0025 * q)) + 0.15; })));
    methods.push_back(new AdaptiveALTMethod(AdaptiveALTParams(20, 0.0, 1.0, 0.0, func)));
    methods.push_back(new AdaptiveALTMethod(AdaptiveALTParams(20, 0.0, 1.0, 0.0, exp_func)));
    methods.push_back(new AdaptiveALTMethod(AdaptiveALTParams(25, 0.0, 1.0, 0.0, exp_func)));
    results.resize(methods.size());

    for (int i = 0; i < numRepeats; i++) {
        std::cout << "Repeat: " << i << std::endl;
        buildIndexes();
        loadQueries();
        runAll();
        queries.clear();
    }
}


void DistanceExperimentCommand::compareThresholdLandmarkAdaptive(int numRepeats)
{
    for (double j = 0.1; j < 0.5; j += 0.03) {
        for (int i = 5; i < 41; i += 5) {
            methods.push_back(new AdaptiveALTMethod(AdaptiveALTParams(i, 0.0, 1.0, 0.0, j)));
        }
        results.resize(methods.size());
        for (int i = 0; i < numRepeats; i++) {
            std::cout << "Repeat: " << i << std::endl;
            buildIndexes();
            loadQueries();
            runAll();
            queries.clear();
        }
        write_to_csv(results, methods, network + "_adapt_" + std::to_string(j) + "_threshold_var_landmark", numRepeats);
        clearMethods();
        results.clear();
    }
}


void DistanceExperimentCommand::compareThresholdLandmark(int numRepeats, LANDMARK_TYPE landmarkType)
{
    for (double j = 0.1; j < 0.5; j += 0.03) {
        for (int i = 5; i < 41; i += 5) {
            methods.push_back(new ALTMethod(i, landmarkType, {j}));
        }
        std::vector<std::vector<double>> results(methods.size());
        for (int i = 0; i < numRepeats; i++) {
            std::cout << "Repeat: " << i << std::endl;
            buildIndexes();
            loadQueries();
            runAll();
            queries.clear();
        }
        write_to_csv(results, methods, network + "_nonadapt_" + std::to_string(j) + "_threshold_var_landmark_" +
                                       std::to_string(landmarkType) + "_type", numRepeats);
        clearMethods();
        results.clear();
    }
}


void DistanceExperimentCommand::compareMethods()
{
    methods.push_back(new ALTMethod(25, LANDMARK_TYPE::RANDOM, {0.12}));
    methods.push_back(new ALTMethod(20, LANDMARK_TYPE::RANDOM, {0.18}));
    methods.push_back(new ALTMethod(15, LANDMARK_TYPE::RANDOM, {0.24}));
    methods.push_back(new ALTMethod(25, LANDMARK_TYPE::AVOID, {0.12}));
    methods.push_back(new ALTMethod(20, LANDMARK_TYPE::AVOID, {0.18}));
    methods.push_back(new ALTMethod(15, LANDMARK_TYPE::AVOID, {0.24}));
    methods.push_back(new ALTMethod(25, LANDMARK_TYPE::MIN_DIST, {0.12}));
    methods.push_back(new ALTMethod(20, LANDMARK_TYPE::MIN_DIST, {0.18}));
    methods.push_back(new ALTMethod(15, LANDMARK_TYPE::MIN_DIST, {0.24}));
    methods.push_back(new AdaptiveALTMethod(AdaptiveALTParams(25, 0.0, 1.0, 0.0, 0.12)));
    methods.push_back(new AdaptiveALTMethod(AdaptiveALTParams(20, 0.0, 1.0, 0.0, 0.18)));
    methods.push_back(new AdaptiveALTMethod(AdaptiveALTParams(15, 0.0, 1.0, 0.0, 0.24)));
    methods.push_back(new AdaptiveALTMethod(
            AdaptiveALTParams(20, 0.0, 1.0, 0.0, [](unsigned q) { return 0.15 * (pow(0.69, 0.0025 * q)) + 0.15; })));
    std::vector<std::vector<double>> results(methods.size());

    for (int i = 0; i < numRepeats; i++) {
        std::cout << "Repeat: " << i << std::endl;
        buildIndexes();
        loadQueries();
        visualizeQueries(network + "_compare_methods");
        runAll();
        exportLandmarks(network + "_compare_methods");
        queries.clear();
    }
    write_to_csv(results, methods, network + "_compare_methods", numRepeats);
    clearMethods();
    results.clear();
}


void DistanceExperimentCommand::execute(int argc, char **argv)
{
    srand(7); // NOLINT(*-msc51-cpp) it is supposed to be possible to reproduce

    std::string bgrFilePath;
    LANDMARK_TYPE landmarkType = LANDMARK_TYPE::MIN_DIST;
    int testNum = -1;
    int opt;
    while ((opt = getopt(argc, argv, "e:g:p:f:s:n:d:t:q:k:m:v:l:r:t:x:")) != -1) {
        switch (opt) {
            case 'g':
                bgrFilePath = optarg;
                break;
            case 'q':
                numQueries = std::stoul(optarg);
                break;
            case 'd':
                maxDist = std::stoul(optarg);
                break;
            case 'k':
                numTargets = std::stoul(optarg);
                break;
            case 't':
                landmarkType = static_cast<LANDMARK_TYPE>(std::stoi(optarg));
                break;
            case 'x':
                testNum = std::stoi(optarg);
                break;
            case 'r':
                numRepeats = std::stoi(optarg);
                break;
            case 'v':
                validate = std::stoi(optarg);
                break;
            case 'f':
                resultsPath = std::string(optarg);
                break;
            default:
                std::cerr << "Unknown option(s) provided!\n\n";
//                std::cout << "Provided option: " << opt << std::endl;
//                showCommandUsage(argv[0]);
//                exit(1);
        }
    }
    if (numQueries == 0 || bgrFilePath.empty() || numTargets == 0) {
        std::cerr << "Missing required arguments!\n\n";
        showCommandUsage(argv[0]);
        exit(1);
    }

    auto indexSlash = bgrFilePath.find_last_of('/');
    auto indexDot = bgrFilePath.find_last_of('.');
    network = bgrFilePath.substr(indexSlash + 1, indexDot - indexSlash - 1);

    std::cout << "** " << network << " **" << std::endl;
    if (!validate) {
        std::cout << "WARNING! Validation is disabled!" << std::endl;
    }
    std::cout << "Number of repeats: " << numRepeats << std::endl;

    graph = serialization::getIndexFromBinaryFile<Graph>(bgrFilePath);


    switch (testNum) {
        case 0:
            runStandardTestCase([this] {
                createMethodsConstABestThreshold();
            });
            break;
        case 1:
            runStandardTestCase([this] {
                createMethodsConstBBestThreshold();
            });
            break;
        case 2:
            runStandardTestCase([this] {
                createMethodsConstCBestThreshold();
            });
            break;
        case 3:
            runStandardTestCase([this] {
                compareDecayFunctions();
            });
            break;
        case 4:
            compareThresholdLandmarkAdaptive(numRepeats);
            break;
        case 5:
            compareThresholdLandmark(numRepeats, landmarkType);
            break;
        case 6:
            runStandardTestCase([this] {
                compareLandmarksNumber();
            });
            break;
        case 7:
            compareNumLandmarksMinDistALT();
            break;
        case 8:
            runStandardTestCase([this] {
                compareThresholdMinDistALT();
            });
            break;
        case 9:
            compareNumLandmarksHopsALT();
            break;
        case 10:
            runStandardTestCase([this] {
                compareThresholdHopsALT();
            });
            break;
        case 11:
            runStandardTestCase([this] {
                compareFarthestALT();
            });
            break;
        case 12:
            runStandardTestCase([this] {
                compareAvoidALT();
            });
            break;
        default:
            compareMethods();
    }

    std::cout << "Finished" << std::endl;
}

void DistanceExperimentCommand::runStandardTestCase(const std::function<void()> &testCase)
{
    results.clear();
    clearMethods();

    testCase();

    write_to_csv(results, methods, resultsPath + "/results", numRepeats);
    results.clear();
    clearMethods();
}

void DistanceExperimentCommand::visualizeQueries(std::string name = "")
{
    std::ofstream file(name + "_query_data.csv");
    Coordinate x, y;
    bool isSource, isTarget;
    file << "x,y,is_source,is_target" << std::endl;
    for (auto graphNode: graph.getNodesIDsVector()) {
        graph.getCoordinates(graphNode, x, y);
        std::string color;
        isSource = false;
        isTarget = false;
        for (auto query: queries) {
            if (query.source == graphNode)
                isSource = true;
            if (std::find(query.targets.begin(), query.targets.end(), graphNode) != query.targets.end())
                isTarget = true;
        }
        file << x << ',' << y << ',' << isSource << ',' << isTarget << std::endl;
    }
    file.close();
}

void DistanceExperimentCommand::exportLandmarks(std::string name = "")
{
    size_t max_size = 0;
    for (const auto &method: methods) {
        max_size = std::max(max_size, method->getLandmarkNodeIDs().size());
    }

    std::vector<std::vector<std::string>> csvData(max_size, std::vector<std::string>(methods.size(), ""));

    Coordinate x, y;
    for (size_t i = 0; i < methods.size(); i++) {
        auto landmarks = methods[i]->getLandmarkNodeIDs();
        for (size_t j = 0; j < landmarks.size(); j++) {
            graph.getCoordinates(landmarks[j], x, y);
            csvData[j][i] = std::to_string(x) + "_" + std::to_string(y);
        }
    }
    std::ofstream file(name + "_landmark_data.csv");
    if (file.is_open()) {
        for (size_t i = 0; i < methods.size(); ++i) {
            file << methods[i]->getInfo();
            if (i < methods.size() - 1) {
                file << ",";
            }
        }
        file << std::endl;
        for (size_t i = 0; i < max_size; ++i) {
            for (size_t j = 0; j < methods.size(); ++j) {
                file << csvData[i][j];
                if (j < methods.size() - 1) {
                    file << ",";
                }
            }
            file << std::endl;
        }
        file.close();
        std::cout << "CSV file written successfully." << std::endl;
    } else {
        std::cerr << "Error opening output file." << std::endl;
    }
}

void DistanceExperimentCommand::showCommandUsage(std::string programName)
{
    std::cerr << "Usage: " << programName << " -c " + constants::DISTANCE_EXPERIMENTS_CMD +
                                             " -g <graph_file_name>" +
                                             " -q <num_of_queries>" +
                                             " -k <k>" +
                                             " \n\n";
}

void DistanceExperimentCommand::buildIndexes(bool saveTimes)
{
    int iter = 0;
    for (auto method: methods) {
        StopWatch sw;
        sw.start();
        method->buildIndex(graph);
        sw.stop();
        if (saveTimes) {
            results[iter].push_back(sw.getTimeMs());
        }
        iter++;
        std::cout << "Time to generate " << method->getName() << " index" << ": " << sw.getTimeMs() << " ms"
                  << std::endl;
    }

}

void DistanceExperimentCommand::loadQueries()
{
//    queries = QueryGenerator().randomExpandTargets(graph, numQueries, 0.000005);
//    queries = QueryGenerator().randomExpandTargetsClustered(graph, numQueries, 3, 0.0001);
    queries = QueryGenerator().random(graph, numQueries, numTargets, std::numeric_limits<unsigned long>::max());
}

void DistanceExperimentCommand::runAll(bool saveOnlyLastResult)
{
    int iter = 0;
    for (auto method: methods) {
        runMethod(method, iter, saveOnlyLastResult);
        iter++;
    }
}

void DistanceExperimentCommand::validateAll()
{
    if (!validate) {
        std::cout << "WARNING: Skipping validation!" << std::endl;
        return;
    }
    std::cout << "*** Validating ***" << std::endl;
//    std::cout << "Validating edge weights..." << std::endl;
//
//    for (auto node: graph.getNodesIDsVector()) {
//        auto edgeStart = graph.getEdgeListStartIndex(node);
//        auto edgeEnd = graph.getEdgeListSize(node);
//
//        for (auto edge = edgeStart; edge < edgeEnd; edge++) {
//            auto edgeNode = graph.edges[edge].first;
//            auto edgeWeight = graph.edges[edge].second;
//            auto euclideanDist = graph.getEuclideanDistance(node, edgeNode);
//
//            if (euclideanDist > edgeWeight) {
//                std::cerr << "Validation failed for edge: " << node << " -> " << edgeNode << std::endl;
//                std::cerr << "Euclidean: " << euclideanDist << ", Edge: " << edgeWeight << std::endl;
//            }
//        }
//    }
//    std::cout << "Finished validating edge weights." << std::endl;


    std::cout << "Validating results..." << std::endl;
    bool validationFailed = false;
    double avgNodesInPath = 0;
    std::vector<NodeID> pathTree(graph.getNumNodes());
    for (auto query: queries) {
        auto source = query.source;
        std::vector<EdgeWeight> expectedDistances;
        for (auto target: query.targets) {
            auto path = DijkstraSearch().findShortestPath(graph, source, target, pathTree);
            avgNodesInPath += path.getNumLinks();
            expectedDistances.push_back(path.getLength());
        }

        std::vector<EdgeWeight> distances(numTargets, 0);

        for (auto method: methods) {
            bool methodFailed = false;
            method->findDistances(graph, query, distances);
            for (unsigned i = 0; i < numTargets; ++i) {
                if (expectedDistances[i] != distances[i]) {
                    methodFailed = true;
                }
            }
            if (methodFailed) {
                std::cout << method->getName() << " failed" << std::endl;
                validationFailed = true;
            }
        }
    }
    avgNodesInPath /= queries.size();

    std::cout << "Average nodes in path: " << avgNodesInPath << std::endl;

    if (!validationFailed) {
        std::cout << "Validation passed!" << std::endl;
    }
}

void DistanceExperimentCommand::runMethod(DistanceMethod *method)
{
    std::cout << "*** Measuring " << method->getName() << "... ***" << std::endl;
    method->printInfo();
    std::vector<EdgeWeight> distances(numTargets, 0);
    double cumulativeTime = 0;
    StopWatch sw;
    sw.start();
    for (unsigned i = 0; true; ++i) {
        if (isPowerOf(i, 2) || queries.size() == i) {
            std::cout << "    " << i << ", " << cumulativeTime << std::endl;
//                method->printStatistics();
            if (i >= queries.size()) {
                break;
            }
        }
        sw.reset();
        sw.start();

        method->findDistances(graph, queries[i], distances);

        sw.stop();
        cumulativeTime += sw.getTimeMs();
    }
    method->printStatistics();
}

void DistanceExperimentCommand::runMethod(DistanceMethod *method, int iter, bool saveOnlyLastResult)
{
    std::cout << "*** Measuring " << method->getName() << "... ***" << std::endl;
    method->printInfo();
    std::vector<EdgeWeight> distances(numTargets, 0);
    double cumulativeTime = 0;
    StopWatch sw;
    sw.start();
    for (unsigned i = 0; true; ++i) {
        if ((isPowerOf(i, 2) && !saveOnlyLastResult) || queries.size() == i) {
            std::cout << "    " << i << ", " << cumulativeTime << std::endl;
            results[iter].push_back(cumulativeTime);
//                method->printStatistics();
            if (i >= queries.size()) {
                break;
            }
        }
        sw.reset();
        sw.start();

        method->findDistances(graph, queries[i], distances);

        sw.stop();
        cumulativeTime += sw.getTimeMs();
    }
    method->printStatistics();
}


//void DistanceExperimentCommand::runMultiTargetALT()
//{
//    alt.edgesAccessedCount = 0;
//
//        alt.findShortestPathDistances(graph, query.source, query.targets);
//    }
////    std::cout << "Edges accessed: " << alt.edgesAccessedCount << std::endl;
//}

void DistanceExperimentCommand::clearMethods()
{
    for (auto method: methods) {
        delete method;
    }
    methods.clear();
}

DistanceExperimentCommand::~DistanceExperimentCommand()
{
    for (auto method: methods) {
        delete method;
    }
}



