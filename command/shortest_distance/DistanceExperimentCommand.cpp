//
// Created by milos on 21/03/2024.
//

#include "DistanceExperimentCommand.h"
#include "../../utility/serialization.h"
#include "../../utility/StopWatch.h"
#include "../../processing/DijkstraSearch.h"
#include "../../processing/AStarSearch.h"


void DistanceExperimentCommand::execute(int argc, char **argv)
{
    srand(1); // NOLINT(*-msc51-cpp) it is supposed to be predictable

    std::string bgrFilePath;
    int opt;
    while ((opt = getopt(argc, argv, "e:g:p:f:s:n:d:t:q:k:m:v:l:r:")) != -1) {
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

    graph = serialization::getIndexFromBinaryFile<Graph>(bgrFilePath);

    methods.push_back(new AdaptiveALTMethod(100));
    methods.push_back(new ALTMethod(20));
//    methods.push_back(new ALTMethod(40));
//    methods.push_back(new AStarMethod());
//    methods.push_back(new DijkstraMethod());
    buildIndexes();
    loadQueries();
    runAll();
    validateAll();
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

void DistanceExperimentCommand::validateAll()
{
    std::cout << "*** Validating ***" << std::endl;
    std::cout << "Validating edge weights..." << std::endl;

    for (auto node: graph.getNodesIDsVector()) {
        auto edgeStart = graph.getEdgeListStartIndex(node);
        auto edgeEnd = graph.getEdgeListSize(node);

        for (auto edge = edgeStart; edge < edgeEnd; edge++) {
            auto edgeNode = graph.edges[edge].first;
            auto edgeWeight = graph.edges[edge].second;
            auto euclideanDist = graph.getEuclideanDistance(node, edgeNode);

            if (euclideanDist > edgeWeight) {
                std::cerr << "Validation failed for edge: " << node << " -> " << edgeNode << std::endl;
                std::cerr << "Euclidean: " << euclideanDist << ", Edge: " << edgeWeight << std::endl;
            }
        }
    }
    std::cout << "Finished validating edge weights." << std::endl;


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

        for(auto method : methods) {
            bool methodFailed = false;
            method->findDistances(graph, query, distances);
            for(unsigned i = 0; i < numTargets; ++i) {
                if(expectedDistances[i] != distances[i]) {
                    methodFailed = true;
                }
            }
            if(methodFailed) {
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
        if(i % 10 == 0) {
            if (i == 10 || i == 50 || i == 100 || i == 500 || i == 1000 || queries.size() == i) {
                sw.stop();
                cumulativeTime += sw.getTimeMs();
                std::cout << "    " << i << ", " << cumulativeTime << std::endl;
                method->printStatistics();
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

