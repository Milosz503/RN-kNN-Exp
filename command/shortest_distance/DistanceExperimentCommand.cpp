//
// Created by milos on 21/03/2024.
//

#include "DistanceExperimentCommand.h"
#include "../../utility/serialization.h"
#include "../../utility/StopWatch.h"
#include "../../processing/DijkstraSearch.h"
#include "../../processing/AStarSearch.h"
#include <fstream>




void write_to_csv(const std::vector<std::vector<double>>& methods, std::string network, int numMeasurements = 9) {
    std::ofstream file(network + "_output.csv");
    std::ofstream file_deviation(network + "_output_dev.csv");

    for (int i = 0; i < methods.size(); ++i) {
        file << "method " << i << ",";
    }
    file << std::endl;

    int size = methods[0].size() / numMeasurements;
    for (int j = 0; j < numMeasurements; j++) {
        for (auto i = 0; i < methods.size(); i++) {
            double avg = 0.0;
            for (int k = 0; k < size; k++) {
                avg += methods[i][k * numMeasurements + j];
            }
            avg /= size;
            double stdder = 0.0;
            for (int k = 0; k < size; k++) {
                stdder += std::pow((methods[i][k * numMeasurements + j] - avg), 2);
            }
            stdder = std::pow((stdder / (size - 1)), 0.5);
            file << avg << ",";
            file_deviation << stdder << ",";
        }
        file << std::endl;
        file_deviation << std::endl;
    }

    file.close();
    file_deviation.close();
}

double func(unsigned queries) {
    if (queries < 2000)
        return queries * (-0.000063) + 0.3;
    return queries * -0.000008 + 0.19;
}

double exp_func(unsigned queries) {
    return 0.15 + 0.25 * std::exp(queries * -0.00157);
}

void DistanceExperimentCommand::clearMethods() {
    for (auto method : methods) {
        delete method;
    }
    methods.clear();
}

void DistanceExperimentCommand::createMethodsConstABestThreshold(int numRepeats, const std::string network, int numQuerySteps){
    for (int i = 0; i < 11; i++) {
        methods.push_back(new AdaptiveALTMethod(AdaptiveALTParams(20, 0.0, i / 10.0, (10 - i) / 10.0, [](unsigned q) { return 0.15 * (pow(0.69, 0.0025 * q)) + 0.15; })));
    }
    std::vector <std::vector<double>> results(methods.size());

    for (int i = 0; i < numRepeats; i++) {
        std::cout << "Repeat: " << i << std::endl;
        buildIndexes(results);
        loadQueries();
        runAll(results);
        queries.clear();
    }
    write_to_csv(results, network + "_const_a", numQuerySteps);
    clearMethods();
    results.clear();
}

void DistanceExperimentCommand::createMethodsConstBBestThreshold(int numRepeats, const std::string network, int numQuerySteps){
    for (int i = 0; i < 11; i++) {
        methods.push_back(new AdaptiveALTMethod(AdaptiveALTParams(20, i / 10.0, 0.0, (10 - i) / 10.0, [](unsigned q) { return 0.15 * (pow(0.69, 0.0025 * q)) + 0.15; })));
    }
    std::vector <std::vector<double>> results(methods.size());

    for (int i = 0; i < numRepeats; i++) {
        std::cout << "Repeat: " << i << std::endl;
        buildIndexes(results);
        loadQueries();
        runAll(results);
        queries.clear();
    }
    write_to_csv(results, network + "_const_b", numQuerySteps);
    clearMethods();
    results.clear();
}

void DistanceExperimentCommand::createMethodsConstCBestThreshold(int numRepeats, const std::string network, int numQuerySteps){
    for (int i = 0; i < 11; i++) {
        methods.push_back(new AdaptiveALTMethod(AdaptiveALTParams(20, i / 10.0, (10 - i) / 10.0, 0.0, [](unsigned q) { return 0.15 * (pow(0.69, 0.0025 * q)) + 0.15; })));
    }
    std::vector <std::vector<double>> results(methods.size());

    for (int i = 0; i < numRepeats; i++) {
        std::cout << "Repeat: " << i << std::endl;
        buildIndexes(results);
        loadQueries();
        runAll(results);
        queries.clear();
    }
    write_to_csv(results, network + "_const_c", numQuerySteps);
    clearMethods();
    results.clear();
}


void DistanceExperimentCommand::compareDecayFunctions(int numRepeats, const std::string network, int numQuerySteps) {
    methods.push_back(new AdaptiveALTMethod(AdaptiveALTParams(20, 0.0, 1.0, 0.0, [](unsigned q) { return 0.15 * (pow(0.69, 0.0025 * q)) + 0.15; })));
    methods.push_back(new AdaptiveALTMethod(AdaptiveALTParams(20, 0.0, 1.0, 0.0, func)));
    methods.push_back(new AdaptiveALTMethod(AdaptiveALTParams(20, 0.0, 1.0, 0.0, exp_func)));
    methods.push_back(new AdaptiveALTMethod(AdaptiveALTParams(25, 0.0, 1.0, 0.0, exp_func)));
    std::vector <std::vector<double>> results(methods.size());

    for (int i = 0; i < numRepeats; i++) {
        std::cout << "Repeat: " << i << std::endl;
        buildIndexes(results);
        loadQueries();
        runAll(results);
        queries.clear();
    }
    write_to_csv(results, network + "_compare_decay", numQuerySteps);
    clearMethods();
    results.clear();
}


void DistanceExperimentCommand::compareThresholdLandmarkAdaptive(int numRepeats, const std::string network, int numQuerySteps) {
    for (double j = 0.1; j < 0.5; j += 0.03) {
        for (int i = 5; i < 41; i += 5) {
            methods.push_back(new AdaptiveALTMethod(AdaptiveALTParams(i, 0.0, 1.0, 0.0, j)));
        }
        std::vector <std::vector<double>> results(methods.size());
        for (int i = 0; i < numRepeats; i++) {
            std::cout << "Repeat: " << i << std::endl;
            buildIndexes(results);
            loadQueries();
            runAll(results);
            queries.clear();
        }
        write_to_csv(results, network + "_adapt_" + std::to_string(j) + "_threshold_var_landmark", numQuerySteps);
        clearMethods();
        results.clear();
    }
}


void DistanceExperimentCommand::compareThresholdLandmark(int numRepeats, const std::string network, LANDMARK_TYPE landmarkType, int numQuerySteps) {
    for (double j = 0.1; j < 0.5; j += 0.03) {
        for (int i = 5; i < 41; i += 5) {
            methods.push_back(new ALTMethod(i, landmarkType, {j}));
        }
        std::vector <std::vector<double>> results(methods.size());
        for (int i = 0; i < numRepeats; i++) {
            std::cout << "Repeat: " << i << std::endl;
            buildIndexes(results);
            loadQueries();
            runAll(results);
            queries.clear();
        }
        write_to_csv(results, network + "_nonadapt_" + std::to_string(j) + "_threshold_var_landmark_" + std::to_string(landmarkType) + "_type", numQuerySteps);
        clearMethods();
        results.clear();
    }
}


void DistanceExperimentCommand::compareMethods(int numRepeats, const std::string network, int numQuerySteps) {
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
    methods.push_back(new AdaptiveALTMethod(AdaptiveALTParams(20, 0.0, 1.0, 0.0, [](unsigned q) { return 0.15 * (pow(0.69, 0.0025 * q)) + 0.15; })));
    std::vector<std::vector<double>> results(methods.size());

    for (int i = 0; i < numRepeats; i++) {
        std::cout << "Repeat: " << i << std::endl;
        buildIndexes(results);
        loadQueries();
        visualizeQueries(network + "_compare_methods");
        runAll(results);
        exportLandmarks(network + "_compare_methods");
        queries.clear();
    }
    write_to_csv(results, network + "_compare_methods", numQuerySteps);
    clearMethods();
    results.clear();
}


void DistanceExperimentCommand::execute(int argc, char **argv) {
    srand(7); // NOLINT(*-msc51-cpp) it is supposed to be possible to reproduce

    std::string bgrFilePath;
    std::string network = "";
    int numRepeats = 0;
    LANDMARK_TYPE landmarkType = LANDMARK_TYPE::MIN_DIST;
    int testNum = 0;
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
            case 'l':
                numLandmarks = std::stoul(optarg);
                break;
            case 'n':
                network = optarg;
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
            default:
                std::cerr << "Unknown option(s) provided!\n\n";
                showCommandUsage(argv[0]);
                exit(1);
        }
    }
    if (numQueries == 0 || bgrFilePath.empty() || maxDist == 0 || numTargets == 0 || numLandmarks == 0) {
        std::cerr << "Missing required arguments!\n\n";
        showCommandUsage(argv[0]);
        exit(1);
    }
    int numQuerySteps = 9;

    graph = serialization::getIndexFromBinaryFile<Graph>(bgrFilePath);

    switch (testNum) {
        case 0:
            createMethodsConstABestThreshold(numRepeats, network, numQuerySteps);
            break;
        case 1:
            createMethodsConstBBestThreshold(numRepeats, network, numQuerySteps);
            break;
        case 2:
            createMethodsConstCBestThreshold(numRepeats, network, numQuerySteps);
            break;
        case 3:
            compareDecayFunctions(numRepeats, network, numQuerySteps);
            break;
        case 4:
            compareThresholdLandmarkAdaptive(numRepeats, network, numQuerySteps);
            break;
        case 5:
            compareThresholdLandmark(numRepeats, network, landmarkType, numQuerySteps);
            break;
        default:
            compareMethods(numRepeats, network, numQuerySteps);
    }
}

void DistanceExperimentCommand::visualizeQueries(std::string name = "") {
    std::ofstream file(name + "_query_data.csv");
    Coordinate x, y;
    bool isSource, isTarget;
    for (auto graphNode : graph.getNodesIDsVector()) {
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

void DistanceExperimentCommand::exportLandmarks(std::string name = "") {
    size_t max_size = 0;
    for (const auto& method : methods) {
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
                                             " -l <num_of_landmarks>" +
                                             " \n\n";
}

void DistanceExperimentCommand::buildIndexes()
{
    for(auto method : methods) {
        StopWatch sw;
        sw.start();
        method->buildIndex(graph);
        sw.stop();
        std::cout << "Time to generate " << method->name << " index" << ": " << sw.getTimeMs() << " ms" << std::endl;
    }

}

void DistanceExperimentCommand::buildIndexes(std::vector<std::vector<double>>& results)
{
    int iter = 0;
    for(auto method : methods) {
        StopWatch sw;
        sw.start();
        method->buildIndex(graph);
        sw.stop();
        results[iter].push_back(sw.getTimeMs());
        iter++;
        std::cout << "Time to generate " << method->name << " index" << ": " << sw.getTimeMs() << " ms" << std::endl;
    }

}

void DistanceExperimentCommand::loadQueries()
{
    std::cout << "Generating queries..." << "numQueries: " << numQueries << " numTargets: " << numTargets << std::endl;


    for (int i = 0; i < numQueries; i++) {
        NodeID start = rand() % graph.getNumNodes();
        Query query;
        query.source = start;
        for (int j = 0; j < numTargets; j++) {
            NodeID end = rand() % graph.getNumNodes();
            auto euclideanDist = graph.getEuclideanDistance(start, end);
            if (euclideanDist > maxDist) {
                j--;
                continue;
            }
            if (std::find(query.targets.begin(), query.targets.end(), end) != query.targets.end()) {
                j--;
                continue;
            }
            query.targets.push_back(end);
        }
        queries.push_back(query);
    }
    std::cout << "Finished generating queries. Number of queries: " << queries.size() << std::endl;
}

void DistanceExperimentCommand::runAll()
{
//    for(int i = 0; i < 3; ++i) {
        for(auto method : methods) {
            runMethod(method);
        }
//    }

//    auto alt  = new AdaptiveALT(graph.getNumNodes(), graph.getNumNodes(), numLandmarks);
//    for(int i = 0; i < 1000; ++i) {
//        std::vector<EdgeWeight> distances(numTargets, 0);
//        StopWatch sw;
//        sw.start();
//        for (auto query: queries) {
//            for (unsigned j = 0; j < query.targets.size(); ++j) {
//                auto target = query.targets[j];
//                distances[j] = alt->findShortestPathDistance(graph, query.source, target);
//            }
//        }
//        sw.stop();
//        std::cout << "Adaptive ALT: " << sw.getTimeMs() << " ms" << std::endl;
//        alt->printStatistics();
//        alt->deleteLowestScoreLandmark();
//        auto node = rand() % graph.getNumNodes();
//        alt->findShortestPathDistance(graph, node, node);
////        auto queryIndex = rand() % queries.size();
////        alt->findShortestPathDistance(graph, queries[queryIndex].source, queries[queryIndex].targets[0]);
//    }


}

void DistanceExperimentCommand::runAll(std::vector<std::vector<double>>& results)
{
    int iter = 0;
    for(auto method : methods) {
        runMethod(method, results, iter);
        iter++;
    }
}

void DistanceExperimentCommand::validateAll()
{
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
                std::cout << method->name << " failed" << std::endl;
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

void DistanceExperimentCommand::runMethod(DistanceMethod* method)
{
    std::cout << "*** Measuring " << method->name << "... ***" << std::endl;
    method->printInfo();
    std::vector<EdgeWeight> distances(numTargets, 0);
    double cumulativeTime = 0;
    StopWatch sw;
    sw.start();
    for(unsigned i = 0; true; ++i) {
        if(i % 4 == 0 || i == 1) {
            if(i == 1 || i == 4 || i == 16 || i == 64 || i == 256 || i == 1024 || i == 4096 || queries.size() == i) {
//            if (i == 10 || i == 50 || i == 100 || i == 500 || i == 1000 || queries.size() == i) {
                sw.stop();
                cumulativeTime += sw.getTimeMs();
                std::cout << "    " << i << ", " << cumulativeTime << std::endl;
//                method->printStatistics();
                sw.reset();
                sw.start();
                if (i >= queries.size()) {
                    break;
                }
            }
        }
        method->findDistances(graph, queries[i], distances);
    }
    method->printStatistics();
}

void DistanceExperimentCommand::runMethod(DistanceMethod* method, std::vector<std::vector<double>>& results, int iter)
{
    std::cout << "*** Measuring " << method->name << "... ***" << std::endl;
    method->printInfo();
    std::vector<EdgeWeight> distances(numTargets, 0);
    double cumulativeTime = 0;
    StopWatch sw;
    sw.start();
    for(unsigned i = 0; true; ++i) {
        if(i % 4 == 0 || i == 1) {
            if(i == 1 || i == 4 || i == 16 || i == 64 || i == 256 || i == 1024 || i == 4096 || queries.size() == i) {
//            if (i == 10 || i == 50 || i == 100 || i == 500 || i == 1000 || queries.size() == i) {
                sw.stop();
                cumulativeTime += sw.getTimeMs();
                results[iter].push_back(cumulativeTime);
                std::cout << "    " << i << ", " << cumulativeTime << std::endl;
//                method->printStatistics();
                sw.reset();
                sw.start();
                if (i >= queries.size()) {
                    break;
                }
            }
        }
        method->findDistances(graph, queries[i], distances);
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

DistanceExperimentCommand::~DistanceExperimentCommand()
{
    for(auto method : methods) {
        delete method;
    }
}

