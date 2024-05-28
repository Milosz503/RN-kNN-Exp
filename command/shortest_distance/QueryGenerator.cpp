//
// Created by milos on 22/05/2024.
//

#include <random>
#include "QueryGenerator.h"

std::vector<Query> QueryGenerator::random(Graph &graph, unsigned int n, unsigned int numTargets, unsigned long maxDist)
{
    std::cout << "Generating queries..." << "numQueries: " << n << " numTargets: " << numTargets << std::endl;
    std::vector<Query> queries;

    for (int i = 0; i < n; i++) {
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
    return queries;
}

std::vector<Query>
QueryGenerator::randomWalkClustered(Graph &graph, unsigned int n, unsigned int numClusters, unsigned int steps)
{
    std::cout << "Generating queries..." << "numQueries: " << n << " steps: " << steps << std::endl;
    std::vector<Query> queries;
    auto clusters = generateClusters(graph, numClusters);

    for (int i = 0; i < n; i++) {
        NodeID cluster = clusters[rand() % clusters.size()];
        NodeID start = randomWalk(graph, cluster, steps);
        NodeID end = randomWalk(graph, start, steps);
        Query query;
        query.source = start;
        query.targets.push_back(end);
        queries.push_back(query);
    }
    std::cout << "Finished generating queries. Number of queries: " << queries.size() << std::endl;
    return queries;
}

std::vector<Query> QueryGenerator::randomWalkTargets(Graph& graph, unsigned int n, unsigned int steps)
{
    std::cout << "Generating queries..." << "numQueries: " << n << " steps: " << steps << std::endl;
    std::vector<Query> queries;

    for (int i = 0; i < n; i++) {
        NodeID start = rand() % graph.getNumNodes();
        Query query;
        query.source = start;
        NodeID end = randomWalk(graph, start, steps);
        query.targets.push_back(end);
        queries.push_back(query);
    }
    std::cout << "Finished generating queries. Number of queries: " << queries.size() << std::endl;
    return queries;
}

std::vector<Query> QueryGenerator::randomExpandTargets(Graph &graph, unsigned int n, double probability)
{
    std::cout << "Generating queries..." << "numQueries: " << n << " probability: " << probability << std::endl;
    std::vector<Query> queries;

    for (int i = 0; i < n; i++) {
        NodeID start = rand() % graph.getNumNodes();
        Query query;
        query.source = start;
        NodeID end = randomExpand(graph, start, probability);
        query.targets.push_back(end);
        queries.push_back(query);
    }
    std::cout << "Finished generating queries. Number of queries: " << queries.size() << std::endl;
    return queries;
}

std::vector<Query>
QueryGenerator::randomExpandTargetsClustered(Graph &graph, unsigned int n, unsigned int numClusters, double probability)
{
    std::cout << "Generating queries..." << "numQueries: " << n << " probability: " << probability << std::endl;
    std::vector<Query> queries;
    auto clusters = generateClusters(graph, numClusters);

    for (int i = 0; i < n; i++) {
        NodeID cluster = clusters[rand() % clusters.size()];
        NodeID start = randomExpand(graph, cluster, probability);
        NodeID end = randomExpand(graph, cluster, probability);
        Query query;
        query.source = start;
        query.targets.push_back(end);
        queries.push_back(query);
    }
    std::cout << "Finished generating queries. Number of queries: " << queries.size() << std::endl;
    return queries;
}


NodeID QueryGenerator::randomWalk(Graph& graph, NodeID source, unsigned int steps)
{
    NodeID current = source;
    for(int i = 0; i < steps; ++i) {
        auto adjListStart = graph.getEdgeListStartIndex(current);
        auto nextAdjListStart = graph.getEdgeListSize(current);
        auto edgeNumber = nextAdjListStart - adjListStart;
        NodeID nextEdge = rand() % edgeNumber;
        current = graph.edges[adjListStart + nextEdge].first;
    }
    return current;
}

NodeID QueryGenerator::randomExpand(Graph &graph, NodeID source, double probability)
{
    std::default_random_engine e1(rand());
    std::bernoulli_distribution pick(probability);
    std::vector<bool> visited(graph.getNumNodes(), false);
    std::deque<NodeDistancePair> queue;
    NodeID nextNode;

    queue.emplace_back(source,0);

    while (!queue.empty()) {
        NodeDistancePair nextElement = queue.front();
        nextNode = nextElement.first;
        auto hops = nextElement.second;
        queue.pop_front();

        if (pick(e1)) {
            return nextNode;
        }

        auto adjListStart = graph.getEdgeListStartIndex(nextNode);
        auto nextAdjListStart = graph.getEdgeListSize(nextNode);
        for (int j = adjListStart; j < nextAdjListStart; ++j) {
            auto adjNode = graph.edges[j].first;
            if (!visited[adjNode]) {
                queue.emplace_back(adjNode, hops + 1);
                visited[adjNode] = true;
            }
        }
    }
    std::cout << "WARNING the furthest node has been selected" << std::endl;
    return nextNode;
}

std::vector<NodeID> QueryGenerator::generateClusters(Graph& graph, unsigned int numClusters)
{
    std::vector<NodeID> clusters;

    for (int i = 0; i < numClusters; i++) {
        clusters.push_back(rand() % graph.getNumNodes());
    }
    return clusters;
}







