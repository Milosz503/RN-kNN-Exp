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
        tempLandmarkDistances(numNodes, 0),
        landmarksQueryAnswered(maxNumLandmarks, 0),
        landmarksQueryNumber(maxNumLandmarks, 0),
//    landmarks(maxNumLandmarks, -1),
    landmarksMaxDistances(maxNumLandmarks, 0),
        numLandmarks(0)
{
    landmarks.reserve(maxNumLandmarks);
//    vertexFromLandmarkDistances.resize(maxNumLandmarks * numNodes);
}

PathDistance AdaptiveALT::findShortestPathDistance(Graph &graph, NodeID source, NodeID target)
{
    auto landmarkQualityS = closestLandmarkQuality(source);
    auto landmarkQualityT = closestLandmarkQuality(target);
    double maxDistanceRatio = 0.20;

    if((landmarkQualityS > maxDistanceRatio || landmarkQualityT > maxDistanceRatio) && numLandmarks < maxNumLandmarks){
        if(landmarkQualityS > landmarkQualityT) {
//            Logger::log("Creating landmark from source ", landmarkQualityS);
            auto landmarkIndex = createLandmark(graph, source);
            return nodeFromLandmarkDistance(landmarkIndex, target);
        }
        else {
//            Logger::log("Creating landmark from target ", landmarkQualityT);
            auto landmarkIndex = createLandmark(graph, target);
            return nodeFromLandmarkDistance(landmarkIndex, source);
        }
    }
    if(landmarkQualityS < landmarkQualityT) {
        return shortestPathDistanceALT(graph, target, source);
    }
    else {
        return shortestPathDistanceALT(graph, source, target);
    }

}

EdgeWeight AdaptiveALT::getLowerBound(NodeID s, NodeID t)
{
    EdgeWeight globalLB = 0, currentLB;
    for (std::size_t i = 0; i < landmarks.size(); ++i) {
        auto landmarkIndex = landmarks[i].index;
        currentLB = std::abs(
                (int)nodeFromLandmarkDistance(landmarkIndex, s) - (int)nodeFromLandmarkDistance(landmarkIndex, t));
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
    landmarks.push_back({landmarkIndex, node, std::vector<EdgeWeight>(numNodes, 0)});
//    landmarks.push_back({landmarkIndex, node});
//    landmarks[landmarkIndex].distances.rehash(numNodes);
//    landmarkDistances.push_back({});
//    landmarkDistances[landmarkIndex].rehash(numNodes);
    numLandmarks++;
    dijkstra.findSSSPDistances(graph, node, tempLandmarkDistances, &tempPqueue);
    EdgeWeight maxDist = 0;
    for (std::size_t j = 0; j < numNodes; ++j) {
        if(tempLandmarkDistances[j] > maxDist) {
            maxDist = tempLandmarkDistances[j];
        }
//        vertexFromLandmarkDistances[j * maxNumLandmarks + landmarkIndex] = tempLandmarkDistances[j];
        landmarks[landmarkIndex].distances[j] = tempLandmarkDistances[j];
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

double AdaptiveALT::closestLandmarkQuality(NodeID node)
{
    double bestQuality = 1;
    for(unsigned i = 0; i < landmarks.size(); ++i) {
        auto landmarkIndex = landmarks[i].index;

        auto quality = nodeFromLandmarkDistance(landmarkIndex, node) / (double)landmarksMaxDistances[landmarkIndex];
        if(quality < bestQuality) {
            bestQuality = quality;
        }
    }
    return bestQuality;
}
