/* Copyright (C) 2015 Tenindra Abeywickrama
 *
 * This file is part of Road Network kNN Experimental Evaluation.
 *
 * Road Network kNN Experimental Evaluation is free software; you can
 * redistribute it and/or modify it under the terms of the GNU Affero 
 * General Public License as published by the Free Software Foundation; 
 * either version 3 of the License, or (at your option) any later version.
 *
 * Road Network kNN Experimental Evaluation is distributed in the hope 
 * that it will be useful, but WITHOUT ANY WARRANTY; without even the 
 * implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR 
 * PURPOSE. See the GNU Affero General Public License for more details.
 *
 * You should have received a copy of the GNU Affero General Public 
 * License along with Road Network kNN Experimental Evaluation; see 
 * LICENSE.txt; if not, see <http://www.gnu.org/licenses/>.
 */

#include "ALT.h"

#include "SetGenerator.h"
#include "DijkstraSearch.h"
#include "AStarSearch.h"
#include "../queue/BinaryMinHeap.h"

#include <iostream>

bool compareObjectListElement(const ObjectListElement &a,
                              const ObjectListElement &b)
{
    return (a.second < b.second);
}

ALT::ALT(std::string _networkName, int _numNodes, int _numEdges) :
        networkName(_networkName), numNodes(_numNodes), numEdges(_numEdges)
{
}

std::string ALT::getNetworkName()
{
    return networkName;
}

unsigned int ALT::getNumEdges()
{
    return numEdges;
}

unsigned int ALT::getNumNodes()
{
    return numNodes;
}

double ALT::computeIndexSize()
{
    double memoryUsage = 0;
    memoryUsage += sizeof(NodeID) * landmarks.size();
    memoryUsage += sizeof(EdgeWeight) * vertexFromLandmarkDistances.size();
    return memoryUsage / (1024 * 1024);
}

void
ALT::buildALT(Graph &graph, LANDMARK_TYPE landmarkType, unsigned int _numLandmarks)
{
    std::vector<NodeID> emptyObjectList;
    buildALT(graph, emptyObjectList, landmarkType, _numLandmarks);
}

void
ALT::buildALT(Graph &graph, std::vector<NodeID> &objectNodes, LANDMARK_TYPE landmarkType, unsigned int _numLandmarks)
{
    unsigned int numNodes = graph.getNumNodes();
    numLandmarks = _numLandmarks;
    if (landmarkType == LANDMARK_TYPE::RANDOM) {
        SetGenerator sg;
        std::vector<NodeID> randomVertices = sg.generateRandomSampleSet(numNodes, numLandmarks);
        landmarks.swap(randomVertices);
    } else {
        std::cerr << "Unknown landmark type" << std::endl;
        std::exit(1);
    }

    // Allocated space for distance from each landmark to each graph vertex
    vertexFromLandmarkDistances.resize(numLandmarks * numNodes);

    // Use Dijkstra's to populate above vector for each landmark
    DijkstraSearch dijk;
    std::vector<EdgeWeight> landmarkDistances(numNodes, 0);
    BinaryMinHeap<EdgeWeight, NodeID> *pqueue = new BinaryMinHeap<EdgeWeight, NodeID>();

    std::vector<ObjectListElement> objectDistances;
    objectDistances.resize(numLandmarks * objectNodes.size());
    for (std::size_t i = 0; i < landmarks.size(); ++i) {

        pqueue->clear();
        dijk.findSSSPDistances(graph, landmarks[i], landmarkDistances, pqueue);
        for (std::size_t j = 0; j < objectNodes.size(); ++j) {
            NodeID object = objectNodes[j];
            objectDistances[i * objectNodes.size() + j] = std::make_pair(object, landmarkDistances[object]);
        }
        auto olStart = objectDistances.begin() + i * objectNodes.size();
        auto olEnd = objectDistances.begin() + (i + 1) * objectNodes.size();
        std::sort(olStart, olEnd, compareObjectListElement);

        for (std::size_t j = 0; j < numNodes; ++j) {
            vertexFromLandmarkDistances[j * numLandmarks + i] = landmarkDistances[j];
            // Note: This will not be cache efficient, because the next vector position
            // we write to will be numLandmarks away. But when we access vertexFromLandmarkDistances
            // vector later, it will be in cache efficient order so queries will be faster.
        }
    }
    delete pqueue;
    objectList.setDistances(objectDistances, objectNodes.size());

//    for(int i = 0; i < 100; i++) {
//        auto element = objectLists[i];
//        std::cout << element.first << " " << element.second << std::endl;
//    }
}

EdgeWeight ALT::getLowerBound(NodeID s, NodeID t)
{
    EdgeWeight globalLB = 0, currentLB;
    for (std::size_t i = 0; i < landmarks.size(); ++i) {
        currentLB = std::abs(
                vertexFromLandmarkDistances[s * numLandmarks + i] - vertexFromLandmarkDistances[t * numLandmarks + i]);
        if (currentLB > globalLB) {
            globalLB = currentLB;
        }
    }
    return globalLB;
}

void ALT::getLowerAndUpperBound(NodeID s, NodeID t, EdgeWeight &lb, EdgeWeight &ub)
{
    EdgeWeight globalLB = 0, globalUB = std::numeric_limits<EdgeWeight>::max(), currentLB, currentUB;
    for (std::size_t i = 0; i < landmarks.size(); ++i) {
        currentLB = std::abs(
                vertexFromLandmarkDistances[s * numLandmarks + i] - vertexFromLandmarkDistances[t * numLandmarks + i]);
        currentUB =
                vertexFromLandmarkDistances[s * numLandmarks + i] - vertexFromLandmarkDistances[t * numLandmarks + i];
        if (currentLB > globalLB) {
            globalLB = currentLB;
        }
        if (currentUB < globalUB) {
            globalUB = currentUB;
        }
    }
}

Path ALT::findShortestPath(Graph &graph, NodeID source, NodeID target,
                           std::vector<NodeID> &shortestPathTree)
{
    // We assume shortestPathTree has been resized for the size of the graph
    // However it does not need to have zero values as we overwrite them

    BinaryMinHeap<EdgeWeight, AStarHeapElement> pqueue;
    std::vector<bool> isNodeSettled(graph.getNumNodes(), false);

//     EdgeWeight adjNodeWgt;
    EdgeWeight minDist, sourceToAdjNodeDist, distanceToTarget = 0;
    NodeID minDistNodeID, adjNode;
    int adjListStart, adjListSize;

    // Initialize priority queue with source node
    EdgeWeight minSourceTargetDist = getLowerBound(source, target);
    pqueue.insert(AStarHeapElement(source, constants::UNUSED_NODE_ID, 0), minSourceTargetDist);

    while (pqueue.size() > 0) {
        // Extract and remove node with smallest possible distance to target
        // and mark it as "settled" so we do not inspect again
        // Note: In A* search this willl still lead to a optimal solution
        // if the heuristic function we use is consistent (i.e. never overestimates)
        AStarHeapElement minElement = pqueue.extractMinElement();
        minDistNodeID = minElement.node;
        if (!isNodeSettled[minDistNodeID]) {
            isNodeSettled[minDistNodeID] = true; // Mark it as "settled" so we can avoid later
            minDist = minElement.sourceNodeDist;
            shortestPathTree[minDistNodeID] = minElement.predecessor;

            if (minDistNodeID == target) {
                // If the minimum is the target we have finished searching
                distanceToTarget = minDist;
                break;
            }

            // Inspect each neighbour and update pqueue using edge weights
            adjListStart = graph.getEdgeListStartIndex(minDistNodeID);
            adjListSize = graph.getEdgeListSize(minDistNodeID);

            for (int i = adjListStart; i < adjListSize; ++i) {
                adjNode = graph.edges[i].first;
                // Only consider neighbours we haven't already settled
                if (!isNodeSettled[adjNode]) {
//                     // Heuristic Consistency Test Output
//                     EdgeWeight currentToTargetEst = getLowerBound(minDistNodeID,target);
//                     EdgeWeight neighbourToTargetEst = getLowerBound(adjNode,target);
//                     if (currentToTargetEst > graph.edges[i].second+neighbourToTargetEst) {
//                         std::cout << "currentToTargetEst = " << currentToTargetEst << std::endl;
//                         std::cout << "adjNodeWgt = " << graph.edges[i].second << std::endl;
//                         std::cout << "neighbourToTargetEst = " << neighbourToTargetEst << std::endl;
//                         std::cout << "Diff = " << currentToTargetEst - graph.edges[i].second - neighbourToTargetEst << std::endl;
//                     }
                    //assert (currentToTargetEst <= adjNodeWgt+neighbourToTargetEst && "Heuristic function is not consistent");

                    sourceToAdjNodeDist = minDist + graph.edges[i].second;
                    minSourceTargetDist = sourceToAdjNodeDist + getLowerBound(adjNode, target);
                    pqueue.insert(AStarHeapElement(adjNode, minDistNodeID, sourceToAdjNodeDist), minSourceTargetDist);
                }
            }
        }
    }

    // Retrieve and return shortest path
    Path path(source, true, distanceToTarget);
    for (NodeID currentNode = target; currentNode != source; currentNode = shortestPathTree[currentNode]) {
        path.addToBeginning(currentNode);
    }

    return path;
}

EdgeWeight ALT::findShortestPathDistance(Graph &graph, NodeID source, NodeID target)
{
    edgesAccessed.clear();

    BinaryMinHeap<EdgeWeight, NodeDistancePair> pqueue;
    std::vector<bool> isNodeSettled(graph.getNumNodes(), false);

//     EdgeWeight adjNodeWgt;
    EdgeWeight minDist, sourceToAdjNodeDist, distanceToTarget = 0;
    NodeID minDistNodeID, adjNode;
    int adjListStart, adjListSize;

    // Initialize priority queue with source node
    EdgeWeight minSourceTargetDist = getLowerBound(source, target);
    pqueue.insert(NodeDistancePair(source, 0), minSourceTargetDist);

    while (pqueue.size() > 0) {
        // Extract and remove node with smallest possible distance to target
        // and mark it as "settled" so we do not inspect again
        // Note: In A* search this willl still lead to a optimal solution
        // if the heuristic function we use is consistent (i.e. never overestimates)
        NodeDistancePair minElement = pqueue.extractMinElement();
        minDistNodeID = minElement.first;
        if (!isNodeSettled[minDistNodeID]) {
            isNodeSettled[minDistNodeID] = true; // Mark it as "settled" so we can avoid later
            minDist = minElement.second;

            if (minDistNodeID == target) {
                // If the minimum is the target we have finished searching
                distanceToTarget = minDist;
                break;
            }

            // Inspect each neighbour and update pqueue using edge weights
            adjListStart = graph.getEdgeListStartIndex(minDistNodeID);
            adjListSize = graph.getEdgeListSize(minDistNodeID);

            for (int i = adjListStart; i < adjListSize; ++i) {
                adjNode = graph.edges[i].first;
                // Only consider neighbours we haven't already settled
                if (!isNodeSettled[adjNode]) {
//                     // Heuristic Consistency Test Output
//                     EdgeWeight currentToTargetEst = getLowerBound(minDistNodeID,target);
//                     EdgeWeight neighbourToTargetEst = getLowerBound(adjNode,target);
//                     if (currentToTargetEst > graph.edges[i].second+neighbourToTargetEst) {
//                         std::cout << "currentToTargetEst = " << currentToTargetEst << std::endl;
//                         std::cout << "adjNodeWgt = " << graph.edges[i].second << std::endl;
//                         std::cout << "neighbourToTargetEst = " << neighbourToTargetEst << std::endl;
//                         std::cout << "Diff = " << currentToTargetEst - graph.edges[i].second - neighbourToTargetEst << std::endl;
//                     }
                    //assert (currentToTargetEst <= adjNodeWgt+neighbourToTargetEst && "Heuristic function is not consistent");

                    sourceToAdjNodeDist = minDist + graph.edges[i].second;
                    minSourceTargetDist = sourceToAdjNodeDist + getLowerBound(adjNode, target);
                    pqueue.insert(NodeDistancePair(adjNode, sourceToAdjNodeDist), minSourceTargetDist);
                }
            }
        }
    }

    edgesAccessedCount += edgesAccessed.size();
    return distanceToTarget;
}

LANDMARK_TYPE ALT::getLandmarkType(std::string selectionMethod, bool &success)
{
    if (selectionMethod == "random") {
        success = true;
        return LANDMARK_TYPE::RANDOM;
    } else {
        // Return default method but indicate method was not found in success
        success = false;
        return LANDMARK_TYPE::RANDOM;
    }
}

NodeID ALT::getClosestCandidate(NodeID q)
{
    pqueue.clear();

    int minDist = vertexFromLandmarkDistances[q * numLandmarks];
    int landmark = 0;
    for(int i = 1; i < numLandmarks; ++i) {
        auto dist = vertexFromLandmarkDistances[q * numLandmarks + i];
        if(dist < minDist) {
            minDist = dist;
            landmark = i;
        }
    }
    queryClosestLandmark = landmark;
    queryLandmarkDistance = minDist;

    int index;
    auto object = objectList.getClosestObject(landmark, minDist, index);
    candidateLeftIndex = index;
    candidateRightIndex = index;
    pqueue.insert(object.first, getLowerBound(q, object.first));

    updateCandidatesQueue(q);

    return pqueue.extractMinElement();
}

NodeID ALT::getNextCandidate(NodeID q, EdgeWeight &lbDistance)
{
    updateCandidatesQueue(q);
    // TODO: handle case no more candidates
    if(pqueue.size() <= 0) {
        throw std::runtime_error("NO MORE CANDIDATES");
    }
    lbDistance = pqueue.getMinKey();
    return pqueue.extractMinElement();
}

void ALT::updateCandidatesQueue(NodeID q)
{
    int newLeftIndex = candidateLeftIndex;
    int newRightIndex = candidateRightIndex;

    while(true) {
        // TODO: optimize: store new candidate
        auto newCandidate = objectList.getNextClosestObject(
                queryClosestLandmark, queryLandmarkDistance, newLeftIndex, newRightIndex);
        if(newCandidate == nullptr) {
            return;
        }
        int lb = std::abs((int)newCandidate->second - vertexFromLandmarkDistances[q * numLandmarks + queryClosestLandmark]);
        if (pqueue.size() == 0 || pqueue.getMinKey() > lb) {
            candidateLeftIndex = newLeftIndex;
            candidateRightIndex = newRightIndex;
            pqueue.insert(newCandidate->first, getLowerBound(q, newCandidate->first));
            continue;
        }

        return;
    }

}
