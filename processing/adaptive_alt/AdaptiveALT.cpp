//
// Created by milos on 09/04/2024.
//

#include "AdaptiveALT.h"
#include "../../utils/Logger.h"
#include "../../utility/StopWatch.h"


AdaptiveALT::AdaptiveALT(int numNodes, int numEdges, int maxNumLandmarks) :
        numNodes(numNodes),
        numEdges(numEdges),
        maxNumLandmarks(maxNumLandmarks),
        landmarksQueryAnswered(maxNumLandmarks, 0),
        landmarksQueryNumber(maxNumLandmarks, 0),
//    landmarks(maxNumLandmarks, -1),
        landmarksMaxDistances(maxNumLandmarks, 0),
        landmarksMaxPaths(maxNumLandmarks, 0),
        numLandmarks(0),
        a(0.0),
        b(0.5),
        c(0.5),
        threshold(0.18)
{
    landmarks.reserve(maxNumLandmarks);
//    vertexFromLandmarkDistances.resize(maxNumLandmarks * numNodes);
}

PathDistance AdaptiveALT::findShortestPathDistance(Graph &graph, NodeID source, NodeID target)
{
//    auto landmarkDistRatioS = closestLandmarkDistanceRatio(source);
    auto landmarkDistRatio = closestLandmarkDistanceRatio(target);
//    auto landmarkDistRatio = std::min(landmarkDistRatioS, landmarkDistRatioT);
    auto landmarkNodesRatio = closestLandmarkNodesRatio(target);
    auto estimatedPathLength = estimatePathLengthRatio(source, target);
    double maxDistanceRatio = 0.185;

    double score = a * landmarkDistRatio + b * landmarkNodesRatio + c * estimatedPathLength * landmarkNodesRatio;

//    auto estimatedPathLength = estimatePathLength(source, target) * landmarkDistanceRatio;
        if(score > threshold) {
//    if(landmarkDistRatioS > maxDistanceRatio && landmarkDistRatioT > maxDistanceRatio) {
//    if ((estimatedPathLength > maxDistanceRatio) && numLandmarks < maxNumLandmarks) {
//        if (landmarkDistRatioS > landmarkDistRatioT) {
//            Logger::log("Creating landmark from source ", landmarkQualityS);
            auto landmarkIndex = createLandmark(graph, target);
//            auto pathLength = landmarks[landmarkIndex].pathLengths[target] / (double)landmarksMaxPaths[landmarkIndex];
//            auto error = pathLength - estimatedPathLength;
//            std::cout << "Path length: " << pathLength << ", estimated: "
//                      << estimatedPathLength << ", error: " << error << std::endl;
            return nodeFromLandmarkDistance(landmarkIndex, source);
//        } else {
////            Logger::log("Creating landmark from target ", landmarkQualityT);
//            auto landmarkIndex = createLandmark(graph, target);
//            auto pathLength = landmarks[landmarkIndex].pathLengths[source] / (double)landmarksMaxPaths[landmarkIndex];
//            auto error = pathLength - estimatedPathLength;
////            std::cout << "Path length: " << pathLength << ", estimated: "
////                      << estimatedPathLength << ", error: " << error << std::endl;
//            return nodeFromLandmarkDistance(landmarkIndex, source);
//        }
    }
//    if (landmarkDistRatioS < landmarkDistRatioT) {
//        return shortestPathDistanceALT(graph, target, source);
//    } else {
        return shortestPathDistanceALT(graph, source, target);
//    }

}

EdgeWeight AdaptiveALT::getLowerBound(NodeID s, NodeID t)
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
    return globalLB;
}

unsigned AdaptiveALT::createLandmark(Graph &graph, NodeID node)
{
    static double cumulativeTime = 0;
//    landmarksQueryNumber[index] = 0;
//    landmarksQueryAnswered[index] = 0;
//    StopWatch sw;
//    sw.start();
    auto landmarkIndex = (unsigned int) landmarks.size();
    landmarks.push_back({landmarkIndex, node,
                         std::vector<EdgeWeight>(numNodes, 0),
                         std::vector<unsigned>(numNodes, 0)
                        });
    auto &landmark = landmarks[landmarkIndex];
//    landmarks.push_back({landmarkIndex, node});
//    landmarks[landmarkIndex].distances.rehash(numNodes);
//    landmarkDistances.push_back({});
//    landmarkDistances[landmarkIndex].rehash(numNodes);
    numLandmarks++;
    dijkstra.findSSSPDistances(graph, node, landmark.distances, landmark.pathLengths, &tempPqueue);
//    dijkstra.findSSSPDistances(graph, node, landmark.distances, &tempPqueue);
    EdgeWeight maxDist = 0;
    unsigned maxPathLength = 0;
    for (std::size_t j = 0; j < numNodes; ++j) {
        if (landmark.distances[j] > maxDist) {
            maxDist = landmark.distances[j];
        }
        if (landmark.pathLengths[j] > maxPathLength) {
            maxPathLength = landmark.pathLengths[j];
        }
//        vertexFromLandmarkDistances[j * maxNumLandmarks + landmarkIndex] = tempLandmarkDistances[j];
        // Note: This will not be cache efficient, because the next vector position
        // we write to will be numLandmarks away. But when we access vertexFromLandmarkDistances
        // vector later, it will be in cache efficient order so queries will be faster.
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

                    sourceToAdjNodeDist = minDist + getEdgeWeight(graph, i);
//                    minSourceTargetDist = sourceToAdjNodeDist + getLowerBound(adjNode, target, bestLandmarks);
                    minSourceTargetDist = sourceToAdjNodeDist + getLowerBound(adjNode, target);
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
    const unsigned numberOfSelectedLandmarks = 5;
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

double AdaptiveALT::estimatePathLengthRatio(NodeID s, NodeID t)
{
    if (numLandmarks == 0) {
        return 1;
    }
    EdgeWeight globalLB = 0, currentLB;
    unsigned bestIndex = 0;
    for (std::size_t i = 0; i < landmarks.size(); ++i) {
        auto landmarkIndex = landmarks[i].index;
        currentLB = std::abs(
                (int) nodeFromLandmarkDistance(landmarkIndex, s) - (int) nodeFromLandmarkDistance(landmarkIndex, t));
        if (currentLB > globalLB) {
            globalLB = currentLB;
            bestIndex = i;
        }
    }
    return std::abs((int) landmarks[bestIndex].pathLengths[s] - (int) landmarks[bestIndex].pathLengths[t])
           /
           (double) landmarksMaxPaths[bestIndex];
}
