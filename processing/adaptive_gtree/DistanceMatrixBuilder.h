//
// Created by milos on 20/02/2024.
//

#ifndef ND_KNN_DISTANCEMATRIXBUILDER_H
#define ND_KNN_DISTANCEMATRIXBUILDER_H


#include "DistanceMatrix.h"
#include "../DijkstraSearch.h"
#include "AdaptiveGtreeNode.h"
#include "../../utils/Logger.h"

class DistanceMatrixBuilder {
public:

    explicit DistanceMatrixBuilder(DijkstraSearch *dijkstra) :
            dijkstra(dijkstra)
    {

    }

    void fillColumn(Graph &graph, DistanceMatrix &matrix, NodeID sourceNodeID, int sourceIdx,
                    const std::vector<NodeID> &targetsVec)
    {
        clearState();
        for (std::size_t j = 0; j < targetsVec.size(); ++j) {
            if (!matrix.isAssigned(j, sourceIdx)) {
                unassignedNodes.push_back(targetsVec[j]);
                nodesMapper.push_back(j);
            }
        }
        calculateDistances(graph, sourceNodeID);
        for (std::size_t j = 0; j < unassignedNodes.size(); ++j) {
            auto weight = distances[unassignedNodes[j]];
            auto row = nodesMapper[j];
            Logger::debug(" > setting weight: ", sourceIdx, ",", row, " ", weight);
            matrix.set(row, sourceIdx, weight);
        }
    }

    // TODO: make a better name
    void fillRow2(Graph &graph, AdaptiveGtreeNode &node, NodeID sourceNodeID, int sourceIdx,
                  const std::vector<NodeID> &targetsVec)
    {
        clearState();
        DistanceMatrix &matrix = node.distanceMatrix;

        for (std::size_t j = 0; j < targetsVec.size(); ++j) {
            auto column = node.getBorderIdxInChildBorderVec(j);
            if (!matrix.isAssigned(sourceIdx, column)) {
                unassignedNodes.push_back(targetsVec[j]);
                nodesMapper.push_back(column);
            }
        }
        calculateDistances(graph, sourceNodeID);

        for (std::size_t j = 0; j < unassignedNodes.size(); ++j) {
            auto weight = distances[unassignedNodes[j]];
            auto column = nodesMapper[j];
            Logger::debug(" > setting weight: ", column, ",", sourceIdx, " ", weight);
            matrix.set(sourceIdx, column, weight);
        }
    }

    // TODO: make a better name
    void fillRow3(Graph &graph, AdaptiveGtreeNode &node, NodeID sourceNodeID, int sourceIdx,
                  const std::vector<NodeID> &targetsVec, int targetOffset)
    {
        clearState();
        DistanceMatrix &matrix = node.distanceMatrix;
        for (std::size_t j = 0; j < targetsVec.size(); ++j) {
            auto column = j + targetOffset;
            if (!matrix.isAssigned(sourceIdx, column)) {
                unassignedNodes.push_back(targetsVec[j]);
                nodesMapper.push_back(column);
            }
        }
        calculateDistances(graph, sourceNodeID);

        for (std::size_t j = 0; j < unassignedNodes.size(); ++j) {
            auto weight = distances[unassignedNodes[j]];
            auto column = nodesMapper[j];
            Logger::debug(" > setting weight: ", column, ",", sourceIdx, " ", weight);
            matrix.set(sourceIdx, column, weight);
        }
    }

    inline void clearState()
    {
        unassignedNodes.clear();
        nodesMapper.clear();
    }

    inline void calculateDistances(Graph &graph, NodeID sourceNodeID)
    {
        distances.clear();
        BinaryMinHeap<EdgeWeight, NodeID> pqueue_local = BinaryMinHeap<EdgeWeight, NodeID>();
        std::unordered_set<NodeID> targetsUset(unassignedNodes.begin(), unassignedNodes.end());
        Logger::debug(" > row not assigned, borders size: ", unassignedNodes.size());
        dijkstra->findSSMTDistances(graph, sourceNodeID, targetsUset, distances, &pqueue_local);
    }

private:
    DijkstraSearch *dijkstra;
    std::unordered_map<NodeID, EdgeWeight> distances;
    std::vector<NodeID> unassignedNodes;
    std::vector<unsigned> nodesMapper;
};


#endif //ND_KNN_DISTANCEMATRIXBUILDER_H
