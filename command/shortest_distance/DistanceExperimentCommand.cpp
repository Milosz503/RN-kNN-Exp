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
    srand(0); // NOLINT(*-msc51-cpp) it is supposed to be predictable

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
    alt.buildALT(graph, LANDMARK_TYPE::RANDOM, numLandmarks);
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
            query.targets.push_back(end);
        }
        queries.push_back(query);
    }
    std::cout << "Finished generating queries. Number of queries: " << queries.size() << std::endl;
}

void DistanceExperimentCommand::runAll()
{
    runMethod([this]() { runDijkstra(); }, "Dijkstra");
    runMethod([this]() { runAStar(); }, "A*");
    runMethod([this]() { runALT(); }, "ALT");
}

void DistanceExperimentCommand::validateAll()
{
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
        for (auto target: query.targets) {
            EdgeWeight dijkstraDist = DijkstraSearch().findShortestPathDistance(graph, source, target);
            EdgeWeight altDist = alt.findShortestPathDistance(graph, query.source, target);
            auto path = DijkstraSearch().findShortestPath(graph, source, target, pathTree);
            avgNodesInPath += path.getNumLinks();
            EdgeWeight astarDist = AStarSearch().findShortestPathDistance(graph, source, target);
            if (dijkstraDist != astarDist) {
                validationFailed = true;
                std::cerr << "Validation failed for query: " << source << " -> " << target << std::endl;
                std::cerr << "Dijkstra: " << dijkstraDist << ", A*: " << astarDist << std::endl;
            }
            if (dijkstraDist != altDist) {
                validationFailed = true;
                std::cerr << "Validation failed for query: " << source << " -> " << target << std::endl;
                std::cerr << "Dijkstra: " << dijkstraDist << ", ALT: " << altDist << std::endl;
            }
        }
    }
    avgNodesInPath /= queries.size();

    std::cout << "Average nodes in path: " << avgNodesInPath << std::endl;

    if (!validationFailed) {
        std::cout << "Validation passed!" << std::endl;
    }
}

void DistanceExperimentCommand::runMethod(std::function<void()> method, std::string methodName)
{
    StopWatch sw;
    sw.start();
    method();
    sw.stop();
    std::cout << methodName << ": " << sw.getTimeMs() << " ms" << std::endl;
}

void DistanceExperimentCommand::runDijkstra()
{
    DijkstraSearch dijkstra;
    for (auto query: queries) {
        for (auto target: query.targets) {
            dijkstra.findShortestPathDistance(graph, query.source, target);
        }
    }
}

void DistanceExperimentCommand::runAStar()
{
    AStarSearch astar;
    for (auto query: queries) {
        for (auto target: query.targets) {
            astar.findShortestPathDistance(graph, query.source, target);
        }
    }
}

void DistanceExperimentCommand::runALT()
{
    for (auto query: queries) {
        for (auto target: query.targets) {
            alt.findShortestPathDistance(graph, query.source, target);
        }
    }
}



