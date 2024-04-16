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
    landmarkDistances(numNodes, 0),
//    landmarksQueryAnswered(maxNumLandmarks, 0),
//    landmarksQueryNumber(maxNumLandmarks, 0),
//    landmarks(maxNumLandmarks, -1),
    landmarksMaxDistances(maxNumLandmarks, 0),
    numLandmarks(0)
{
    landmarks.reserve(maxNumLandmarks);
    vertexFromLandmarkDistances.resize(maxNumLandmarks * numNodes);
}

bool AdaptiveALT::shouldCreateLandmark(NodeID node)
{
    return closestLandmarkQuality(node) > 0.33 && numLandmarks < maxNumLandmarks;
}

PathDistance AdaptiveALT::findShortestPathDistance(Graph &graph, NodeID source, NodeID target)
{
    if(shouldCreateLandmark(source)) {
        Logger::log("Creating landmark from source ", closestLandmarkQuality(source));
        auto landmarkIndex = createLandmark(graph, source);
        return nodeFromLandmarkDistance(landmarkIndex, target);
    }
    if(shouldCreateLandmark(target)) {
        Logger::log("Creating landmark from target ", closestLandmarkQuality(target));
        auto landmarkIndex = createLandmark(graph, target);
        return nodeFromLandmarkDistance(landmarkIndex, source);
    }
    return shortestPathDistanceALT(graph, source, target);
}

EdgeWeight AdaptiveALT::getLowerBound(NodeID s, NodeID t)
{
    EdgeWeight globalLB = 0, currentLB;
    std::size_t bestLandmarkIndex = 0;
    for (std::size_t i = 0; i < landmarks.size(); ++i) {
//        if(landmarks[i] < 0) {
//            continue;
//        }
        auto landmarkIndex = landmarks[i].index;
//        landmarksQueryNumber[i]++;
        currentLB = std::abs(
                vertexFromLandmarkDistances[s * maxNumLandmarks + landmarkIndex] - vertexFromLandmarkDistances[t * maxNumLandmarks + landmarkIndex]);
        if (currentLB > globalLB) {
            globalLB = currentLB;
//            bestLandmarkIndex = i;
        }
    }
//    landmarksQueryAnswered[bestLandmarkIndex]++;
    return globalLB;
}

unsigned AdaptiveALT::createLandmark(Graph &graph, NodeID node)
{
    static double cumulativeTime = 0;
//    unsigned int index = 0;
//    for(unsigned i = 0; i < landmarks.size(); ++i) {
//        if(landmarks[i] < 0) {
//            index = i;
//            break;
//        }
//    }
//    landmarks[index] = node;
//    landmarksQueryNumber[index] = 0;
//    landmarksQueryAnswered[index] = 0;
//    StopWatch sw;
//    sw.start();
    auto landmarkIndex = (unsigned int) landmarks.size();
    landmarks.push_back({landmarkIndex, node});
    numLandmarks++;
    dijkstra.findSSSPDistances(graph, node, landmarkDistances, &pqueue);
    EdgeWeight maxDist = 0;
    for (std::size_t j = 0; j < numNodes; ++j) {
        if(landmarkDistances[j] > maxDist) {
            maxDist = landmarkDistances[j];
        }
        vertexFromLandmarkDistances[j * maxNumLandmarks + landmarkIndex] = landmarkDistances[j];
        // Note: This will not be cache efficient, because the next vector position
        // we write to will be numLandmarks away. But when we access vertexFromLandmarkDistances
        // vector later, it will be in cache efficient order so queries will be faster.
    }
    landmarksMaxDistances[landmarkIndex] = maxDist;
//    sw.stop();
//    cumulativeTime += sw.getTimeMs();
//    std::cout << "Time to create landmarks: " << cumulativeTime << " ms" << std::endl;

    return landmarkIndex;
}

PathDistance AdaptiveALT::shortestPathDistanceALT(Graph &graph, NodeID source, NodeID target)
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

                    sourceToAdjNodeDist = minDist + getEdgeWeight(graph, i);
                    minSourceTargetDist = sourceToAdjNodeDist + getLowerBound(adjNode, target);
                    pqueue.insert(NodeDistancePair(adjNode, sourceToAdjNodeDist), minSourceTargetDist);
                }
            }
        }
    }

    edgesAccessedCount += edgesAccessed.size();
    return distanceToTarget;
}

//void AdaptiveALT::deleteLowestScoreLandmark()
//{
//    int lowestScoreIndex = -1;
//    double lowestScore = 1;
//
//    for(unsigned i = 0; i < landmarks.size(); ++i) {
//        if(landmarks[i] < 0) {
//            continue;
//        }
//        if(landmarkScore(i) < lowestScore) {
//            lowestScore = landmarkScore(i);
//            lowestScoreIndex = i;
//        }
//    }
//
//    if(lowestScoreIndex >= 0) {
//        std::cout << " Deleting " << lowestScoreIndex << "... \n";
//        numLandmarks--;
//        landmarks[lowestScoreIndex] = -1;
//        landmarksQueryNumber[lowestScoreIndex] = 0;
//        landmarksQueryAnswered[lowestScoreIndex] = 0;
//
//        for(unsigned i = 0; i < landmarks.size(); ++i) {
//            landmarksQueryNumber[i] = 0;
//            landmarksQueryAnswered[i] = 0;
//        }
//    }
//}

double AdaptiveALT::closestLandmarkQuality(NodeID node)
{
    double bestQuality = 1;
    for(unsigned i = 0; i < landmarks.size(); ++i) {
//        if (landmarks[i] < 0) {
//            continue;
//        }
        auto landmarkIndex = landmarks[i].index;

        auto quality = nodeFromLandmarkDistance(landmarkIndex, node) / (double)landmarksMaxDistances[landmarkIndex];
        if(quality < bestQuality) {
            bestQuality = quality;
        }
    }
    return bestQuality;
}
