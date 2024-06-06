//
// Created by milos on 22/05/2024.
//

#ifndef ND_KNN_QUERYGENERATOR_H
#define ND_KNN_QUERYGENERATOR_H


#include "../../common.h"
#include "DistanceMethod.h"

class QueryGenerator {
public:

    std::vector<Query> random(Graph& graph, unsigned n, unsigned numTargets, unsigned long maxDist);

    std::vector<Query> randomWalkClustered(Graph& graph, unsigned n, unsigned numClusters, unsigned steps);

    std::vector<Query> randomWalkTargets(Graph& graph, unsigned n, unsigned steps);

    std::vector<Query> randomExpandTargets(Graph& graph, unsigned n, double probability);

    std::vector<Query> randomExpandTargetsClustered(Graph& graph, unsigned n, unsigned numClusters, double probability);

    NodeID randomWalk(Graph& graph, NodeID source, unsigned steps);

    NodeID randomExpand(Graph& graph, NodeID source, double probability);

    std::vector<NodeID> generateClusters(Graph& graph, unsigned numClusters);

    std::vector<NodeID> randomObjects(Graph &graph, double density);

    std::vector<NodeID> randomKNN(Graph &graph, unsigned int n);

    std::vector<NodeID>
    randomExpandObjectsClustered(Graph &graph, double density, unsigned int numClusters, double probability);

    std::vector<NodeID>
    randomExpandKNNQueriesClustered(Graph &graph, unsigned int n, unsigned int numClusters, double probability);
};


#endif //ND_KNN_QUERYGENERATOR_H
