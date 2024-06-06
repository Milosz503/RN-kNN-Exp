//
// Created by milos on 09/04/2024.
//

#include "AdaptiveALT.h"
#include "../../utils/Logger.h"
#include "../../utility/StopWatch.h"


AdaptiveALT::AdaptiveALT(int numNodes, int numEdges, AdaptiveALTParams &params) :
        numNodes(numNodes),
        numEdges(numEdges),
        maxNumLandmarks(params.maxLandmarks),
        landmarksQueryAnswered(maxNumLandmarks, 0),
        landmarksQueryNumber(maxNumLandmarks, 0),
//    landmarks(maxNumLandmarks, -1),
#ifndef DYNAMIC_LANDMARKS_A_ALT
        vertexFromLandmarkDistances(maxNumLandmarks * numNodes, 0),
#endif
        landmarksMaxDistances(maxNumLandmarks, 0),
        landmarksMaxPaths(maxNumLandmarks, 0),
        numLandmarks(0),
        params(params),
        tempLandmarkDistances(numNodes, 0)
{
    landmarks.reserve(maxNumLandmarks);
//    vertexFromLandmarkDistances.resize(maxNumLandmarks * numNodes);
}

void AdaptiveALT::refineIndex(Graph &graph, NodeID source, NodeID target)
{
    queryNumber++;


    auto landmarkDistRatio = closestLandmarkDistanceRatio(target);
    auto landmarkNodesRatio = closestLandmarkNodesRatio(target);
    double estimatedPathLength = 1;
    if (numLandmarks > 0) {
        estimatedPathLength = estimatePathLengthRatio(source, target) * landmarkDistRatio
                              - landmarks[findClosestLandmark(source)].pathLengths[target] / (double)numNodes;
    }

    double score = params.a * landmarkDistRatio +
                   params.b * landmarkNodesRatio +
                   params.c * estimatedPathLength;
    double threshold = params.thresholdFunction(queryNumber);

    if (score > threshold && numLandmarks < maxNumLandmarks) {
        StopWatch sw;
        sw.start();
        createLandmark(graph, target);
        sw.stop();
        landmarkCreateTime += sw.getTimeMs();
    }
}

PathDistance AdaptiveALT::findShortestPathDistance(Graph &graph, NodeID source, NodeID target)
{
    queryNumber++;


    auto landmarkDistRatio = closestLandmarkDistanceRatio(target);
    auto landmarkNodesRatio = closestLandmarkNodesRatio(target);
    double estimatedPathLength = 1;
    if (numLandmarks > 0) {
        estimatedPathLength = estimatePathLengthRatio(source, target) * landmarkDistRatio
                              - landmarks[findClosestLandmark(source)].pathLengths[target] / (double)numNodes;
        estimatedPathLength = std::max(estimatedPathLength, 0.0);
    }

    double score = params.a * landmarkDistRatio +
                   params.b * landmarkNodesRatio +
                   params.c * estimatedPathLength;
    double threshold = params.thresholdFunction(queryNumber);


    if (score > threshold && numLandmarks < maxNumLandmarks) {
        StopWatch sw;
        sw.start();
        auto landmarkIndex = createLandmark(graph, target);
        sw.stop();
        landmarkCreateTime += sw.getTimeMs();
//        auto nodeDist = nodeFromLandmarkDistance(landmarkIndex, source) / (double)landmarksMaxDistances[landmarkIndex];
//        auto err = std::abs(nodeDist - estimatedPathLength);
//        cumulativeEstimateError += err;
//        std::cout << "Dist from landmark " <<  landmarkNodesRatio << std::endl;
//        std::cout << "Estimated error: " <<  nodeDist << ", " << estimatedPathLength << ", " << err << std::endl;
//        std::cout << "Avg error: " << cumulativeEstimateError / numLandmarks << std::endl;
        return nodeFromLandmarkDistance(landmarkIndex, source);
//        auto result = shortestPathDistanceALT(graph, target, source);
//
//
//        return result;
    }

    auto result = shortestPathDistanceALT(graph, source, target);
//    if (queryNumber % 10 == 0) {
//    if (numLandmarks > 0) {
//        estimationCount++;
//        double ratioVisitedNodes = nodesVisited / (double) graph.getNumNodes();
//        std::cout << "Nodes visited: " << nodesVisited << ", ratio: " << ratioVisitedNodes
//                  << std::endl;
//        double estimatedNodes =
//                estimatedPathLength;
//        //estimatedPathLength * estimatedPathLength * M_PI;
//        std::cout << "Estimated nodes visited: " << estimatedNodes << std::endl;
//        auto err = std::abs(ratioVisitedNodes - estimatedNodes);
//        cumulativeEstimateError += err;
//        std::cout << "Avg error: " << cumulativeEstimateError / estimationCount << std::endl;
//    }
//    }
//    double ratioVisitedNodes = nodesVisited / (double) graph.getNumNodes();
//    if(ratioVisitedNodes > 0.2) {
//        std::cout << "For query " << queryNumber << " visited nodes: " << ratioVisitedNodes << std::endl;
//    }
    return result;
}

EdgeWeight AdaptiveALT::getLowerBound(Graph& graph, NodeID s, NodeID t)
{
    EdgeWeight globalLB = 0, currentLB;
    for (std::size_t i = 0; i < landmarks.size(); ++i) {
        auto landmarkIndex = landmarks[i].index;
        currentLB = std::abs(
                (int) nodeFromLandmarkDistance(landmarkIndex, s) - (int) nodeFromLandmarkDistance(landmarkIndex, t));
        if (currentLB > globalLB) {
            globalLB = currentLB;
        }
    }
#ifdef USE_EUCLIDEAN
    auto euclideanDist = static_cast<EdgeWeight>(graph.getEuclideanDistance(s, t) * graph.getMinGraphSpeedByEdge());
    if(euclideanDist > globalLB) {
//        euclideanCounter++;
        return euclideanDist;
    }
    else {
//        landmarkCounter++;
        return globalLB;
    }
#else
    return globalLB;
#endif
}

unsigned AdaptiveALT::createLandmark(Graph &graph, NodeID node)
{
    if(params.results != nullptr) {
        params.results->push_back(queryNumber);
    }

    static double cumulativeTime = 0;
//    StopWatch sw;
//    sw.start();
    auto landmarkIndex = (unsigned int) landmarks.size();
    landmarks.push_back(Landmark(landmarkIndex, node, numNodes));
    auto &landmark = landmarks[landmarkIndex];
    numLandmarks++;
#ifdef DYNAMIC_LANDMARKS_A_ALT
    dijkstra.findSSSPDistances(graph, node, landmark.distances, landmark.pathLengths, &tempPqueue);
#else
#ifdef ESTIMATE_VISITED_NODES
    dijkstra.findSSSPDistances(graph, node, tempLandmarkDistances, landmark.pathLengths, landmark.nodesVisited,
                               &tempPqueue);
#else
    dijkstra.findSSSPDistances(graph, node, tempLandmarkDistances, landmark.pathLengths,
                               &tempPqueue);
#endif // ESTIMATE_VISITED_NODES
#endif // DYNAMIC_LANDMARKS_A_ALT
    EdgeWeight maxDist = 0;
    unsigned maxPathLength = 0;
    for (std::size_t j = 0; j < numNodes; ++j) {
#ifndef DYNAMIC_LANDMARKS_A_ALT
        vertexFromLandmarkDistances[j * maxNumLandmarks + landmarkIndex] = tempLandmarkDistances[j];
        // Note: This will not be cache efficient, because the next vector position
        // we write to will be numLandmarks away. But when we access vertexFromLandmarkDistances
        // vector later, it will be in cache efficient order so queries will be faster.
#endif
        if (nodeFromLandmarkDistance(landmarkIndex, j) > maxDist) {
            maxDist = nodeFromLandmarkDistance(landmarkIndex, j);
        }
        if (landmark.pathLengths[j] > maxPathLength) {
            maxPathLength = landmark.pathLengths[j];
        }
    }
    landmarksMaxDistances[landmarkIndex] = maxDist;
    landmarksMaxPaths[landmarkIndex] = maxPathLength;
//    sw.stop();
//    cumulativeTime += sw.getTimeMs();
//    std::cout << "Time to create landmarks: " << cumulativeTime << " ms" << std::endl;

    return landmarkIndex;
}

PathDistance AdaptiveALT::shortestPathDistanceALT(Graph &graph, NodeID source, NodeID target)
{
    nodesVisited = 0;
    edgesAccessed.clear();

//    auto bestLandmarks = selectBestLandmarks(source, target);

    BinaryMinHeap<EdgeWeight, NodeDistancePair> pqueue;
    std::vector<bool> isNodeSettled(graph.getNumNodes(), false);

//     EdgeWeight adjNodeWgt;
    EdgeWeight minDist, sourceToAdjNodeDist, distanceToTarget = 0;
    NodeID minDistNodeID, adjNode;
    int adjListStart, adjListSize;

    // Initialize priority queue with source node
//    EdgeWeight minSourceTargetDist = getLowerBound(source, target, bestLandmarks);
    EdgeWeight minSourceTargetDist = getLowerBound(graph, source, target);
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
            nodesVisited++;
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

                    sourceToAdjNodeDist = minDist + getEdgeWeight(graph, i);
//                    minSourceTargetDist = sourceToAdjNodeDist + getLowerBound(adjNode, target, bestLandmarks);
                    minSourceTargetDist = sourceToAdjNodeDist + getLowerBound(graph, adjNode, target);
                    pqueue.insert(NodeDistancePair(adjNode, sourceToAdjNodeDist), minSourceTargetDist);
                }
            }
        }
    }

    edgesAccessedCount += edgesAccessed.size();
    return distanceToTarget;
}

double AdaptiveALT::closestLandmarkDistanceRatio(NodeID node)
{
    double bestRatio = 1;
    for (unsigned i = 0; i < landmarks.size(); ++i) {
        auto landmarkIndex = landmarks[i].index;

        auto ratio = nodeFromLandmarkDistance(landmarkIndex, node) / (double) landmarksMaxDistances[landmarkIndex];
//        auto quality = landmarks[landmarkIndex].pathLengths[node] / (double) landmarksMaxPaths[landmarkIndex];
        if (ratio < bestRatio) {
            bestRatio = ratio;
        }
    }
    return bestRatio;
}

double AdaptiveALT::closestLandmarkNodesRatio(NodeID node)
{
    double bestRatio = 1;
    for (unsigned i = 0; i < landmarks.size(); ++i) {
        auto landmarkIndex = landmarks[i].index;

//        auto quality = nodeFromLandmarkDistance(landmarkIndex, node) / (double) landmarksMaxDistances[landmarkIndex];
        auto ratio = landmarks[landmarkIndex].pathLengths[node] / (double) landmarksMaxPaths[landmarkIndex];
        if (ratio < bestRatio) {
            bestRatio = ratio;
        }
    }
    return bestRatio;
}

std::vector<unsigned> AdaptiveALT::selectBestLandmarks(NodeID s, NodeID t)
{
    const unsigned numberOfSelectedLandmarks = 7;
    std::vector<unsigned> bestLandmarks(numberOfSelectedLandmarks, 0);
    std::vector<unsigned> bestBounds(numberOfSelectedLandmarks, 0);

    EdgeWeight currentLB;
    for (std::size_t i = 0; i < landmarks.size(); ++i) {
        auto landmarkIndex = landmarks[i].index;
        currentLB = std::abs(
                (int) nodeFromLandmarkDistance(landmarkIndex, s) - (int) nodeFromLandmarkDistance(landmarkIndex, t));
        if (currentLB > bestBounds.back()) {
            unsigned index = i;
            for (unsigned j = 0; j < numberOfSelectedLandmarks; ++j) {
                if (currentLB > bestBounds[j]) {
                    std::swap(bestBounds[j], currentLB);
                    std::swap(bestLandmarks[j], index);
                }
            }
        }
    }

    if (numLandmarks < numberOfSelectedLandmarks) {
        return std::vector<unsigned>(bestLandmarks.begin(), bestLandmarks.begin() + numLandmarks);
    }
    return bestLandmarks;
}

EdgeWeight AdaptiveALT::getLowerBound(NodeID s, NodeID t, std::vector<unsigned int> &landmarkIndexes)
{
    EdgeWeight globalLB = 0, currentLB;
    for (auto index: landmarkIndexes) {
        currentLB = std::abs(
                (int) nodeFromLandmarkDistance(index, s) - (int) nodeFromLandmarkDistance(index, t));
        if (currentLB > globalLB) {
            globalLB = currentLB;
        }
    }
    return globalLB;
}

unsigned AdaptiveALT::findClosestLandmark(NodeID t)
{
    EdgeWeight bestDist = nodeFromLandmarkDistance(0, t);
    double dist;
    unsigned bestIndex = 0;
    for (std::size_t i = 0; i < landmarks.size(); ++i) {
        auto landmarkIndex = landmarks[i].index;
        dist = nodeFromLandmarkDistance(landmarkIndex, t);
        if (dist < bestDist) {
            bestDist = dist;
            bestIndex = i;
        }
    }
    return bestIndex;
}

double AdaptiveALT::estimatePathLengthRatio(NodeID s, NodeID t)
{
#ifdef ESTIMATE_VISITED_NODES
    if (numLandmarks == 0) {
        return 1;
    }
    return landmarks[findClosestLandmark(s)].nodesVisited[t] / (double) numNodes;
#else
    return 0;
#endif

//    EdgeWeight globalLB = 0, currentLB;
//    unsigned selectedLandmark = 0;
//    for (std::size_t i = 0; i < landmarks.size(); ++i) {
//        auto landmarkIndex = landmarks[i].index;
//        currentLB = std::abs(
//                (int) nodeFromLandmarkDistance(landmarkIndex, s) - (int) nodeFromLandmarkDistance(landmarkIndex, t));
//        if (currentLB > globalLB) {
//            globalLB = currentLB;
//            selectedLandmark = i;
//        }
//    }
//    return globalLB / (double) landmarksMaxDistances[selectedLandmark];

//    EdgeWeight closestDist = landmarks[0].pathLengths[t];
//    unsigned closestLandmark = 0;
//    for (std::size_t i = 0; i < landmarks.size(); ++i) {
//
//        auto currentLB = std::abs(
//                (int) nodeFromLandmarkDistance(i, s) - (int) nodeFromLandmarkDistance(i, t));
//
//        if (closestDist > currentLB) {
//            closestDist = currentLB;
//            closestLandmark = i;
//        }
//    }
//    return std::abs((int) landmarks[closestLandmark].pathLengths[s] - (int) landmarks[closestLandmark].pathLengths[t])
//           /
//           (double) landmarksMaxPaths[closestLandmark];
}


