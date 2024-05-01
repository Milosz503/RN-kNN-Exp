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
#include <cmath>

bool compareObjectListElement(const ObjectListElement &a,
                              const ObjectListElement &b)
{
    return (a.second < b.second);
}

ALT::ALT(std::string _networkName, int _numNodes, int _numEdges, ALTParameters parameters) :
        networkName(_networkName), numNodes(_numNodes), numEdges(_numEdges), parameters(parameters)
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

//// Recursively calculate size of each node
unsigned long ALT::calculateSizes(
        NodeID root,
        std::vector<std::tuple<std::vector<NodeID>, EdgeWeight, unsigned long>>& tree,
        std::vector<NodeID>& landmarks
) {
    EdgeWeight weight = std::get<1>(tree[root]);
    unsigned long size = weight;

    for (auto child : std::get<0>(tree[root])) {
        if (std::find(landmarks.begin(), landmarks.end(), child) == landmarks.end())
            size += calculateSizes(child, tree, landmarks);
    }

    tree[root] = std::make_tuple(std::get<0>(tree[root]), weight, size);

    return size;

}

SizeNumNodesPair ALT::calculateSizes(
        NodeID root,
        std::vector<std::tuple<std::vector<NodeID>, EdgeWeight, unsigned long, unsigned>>& tree,
        std::vector<NodeID>& landmarks
) {
    EdgeWeight weight = std::get<1>(tree[root]);
    SizeNumNodesPair size_num_nodes(weight, std::get<0>(tree[root]).size());
    if (size_num_nodes.numNodes != 0u) {
        for (auto child : std::get<0>(tree[root])) {
            if (std::find(landmarks.begin(), landmarks.end(), child) == landmarks.end())
                size_num_nodes += calculateSizes(child, tree, landmarks);
        }
        tree[root] = std::make_tuple(std::get<0>(tree[root]), weight, size_num_nodes.size, size_num_nodes.numNodes);
        return size_num_nodes;
    }
    tree[root] = std::make_tuple(std::get<0>(tree[root]), weight, size_num_nodes.size, 1u);
    return size_num_nodes;
}

NodeID ALT::getMaxLeaf(
        NodeID root,
        std::vector<std::tuple<std::vector<NodeID>, EdgeWeight, unsigned long>>& tree
) {
    unsigned long maxSize = 0ul;
    NodeID maxLeaf = root;
    unsigned long childSize = 0ul;
    while (true) {
        if (std::get<0>(tree[root]).size() == 0)
            break;
        for (NodeID child : std::get<0>(tree[root])) {
            childSize = std::get<2>(tree[child]);
            if (childSize >= maxSize) {
                maxSize = childSize;
                maxLeaf = child;
            }
        }
        maxSize = 0ul;
        root = maxLeaf;
    }
    return maxLeaf;
}

NodeID ALT::getMaxLeaf(
        NodeID root,
        std::vector<std::tuple<std::vector<NodeID>, EdgeWeight, unsigned long, unsigned>>& tree
) {
    double maxSize = 0.0;
    NodeID maxLeaf = root;
    double childSize = 0.0;
    while (true) {
        if (std::get<0>(tree[root]).size() == 0)
            break;
        for (NodeID child : std::get<0>(tree[root])) {
            childSize = std::get<2>(tree[child]) * std::get<3>(tree[child]);
            if (childSize >= maxSize) {
                maxSize = childSize;
                maxLeaf = child;
            }
        }
        maxSize = 0.0;
        root = maxLeaf;
    }
    return maxLeaf;
}


void
ALT::buildALT(Graph &graph, std::vector<NodeID> &objectNodes, LANDMARK_TYPE landmarkType, unsigned int _numLandmarks)
{
    unsigned int numNodes = graph.getNumNodes();
    this->numNodes = graph.getNumNodes();
    this->numEdges = graph.getNumEdges();
    numLandmarks = _numLandmarks;

    // Use Dijkstra's to populate above vector for each landmark
    DijkstraSearch dijk;
    std::vector<EdgeWeight> landmarkDistances(numNodes, 0);
    BinaryMinHeap<EdgeWeight, NodeID> *pqueue = new BinaryMinHeap<EdgeWeight, NodeID>();

    vertexFromLandmarkDistances.resize(numLandmarks * numNodes);

    std::vector<ObjectListElement> objectDistances;
    objectDistances.resize(numLandmarks * objectNodes.size());

    if (landmarkType == LANDMARK_TYPE::RANDOM) {
        SetGenerator sg;
        std::vector<NodeID> randomVertices = sg.generateRandomSampleSet(numNodes, numLandmarks);
        landmarks.swap(randomVertices);
    } else if (landmarkType == LANDMARK_TYPE::RANDOM_OBJECTS) {
        SetGenerator sg;
        std::vector<NodeID> randomVertices = sg.generateRandomSampleSet(objectNodes.size(), numLandmarks);
        landmarks.clear();
        landmarks.reserve(randomVertices.size());
        for(auto vert : randomVertices) {
            landmarks.push_back(objectNodes[vert]);
        }
    } else if (landmarkType == LANDMARK_TYPE::MIN_DIST) {
        generateMinDistLandmarks(graph, numLandmarks);
        return;
    }
    else if (landmarkType == LANDMARK_TYPE::AVOID) {
        SetGenerator sg;
        //// Number to divide the number of total landmarks
        int divideBy = 10;
        //// get the rounded up number of divided landmarks
        int numInitialLandmarks = std::ceil(numLandmarks / divideBy);
        std::vector<NodeID> randomSet = sg.generateRandomSampleSet(objectNodes.size(), numLandmarks);
        //// init first a certain % of landmark starting points
        std::vector<NodeID> initialLandmarkVertices(randomSet.begin(), randomSet.begin() + numInitialLandmarks);
        //// init random vertices from which we want to run avoid in every iteration to find a landmark candidate
        std::vector<NodeID> randomVertices(randomSet.begin() + numInitialLandmarks, randomSet.end());

        landmarks.clear();
        landmarks.reserve(randomSet.size());

        for (auto vert : initialLandmarkVertices) {
            landmarks.push_back(objectNodes[vert]);
        }

        //// init the first landmarks - de facto the random objects approach
        std::size_t startingIndex = initialLandmarkVertices.size();
        for (std::size_t i = 0; i < initialLandmarkVertices.size(); ++i) {
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
            }
        }

        //// Shortest path tree as a vector of tuples of children ids, weights, and sizes
        std::vector<std::tuple<std::vector<NodeID>, EdgeWeight, unsigned long>> spTree(numNodes);
        int adjListStart, nextAdjListStart;
        NodeID randomVertex;
        EdgeWeight vertexWeight;

        for (std::size_t i = startingIndex; i < numLandmarks; ++i) {
            pqueue->clear();
            //// randomVertex should be indexed from 0
            randomVertex = objectNodes[randomVertices[i - numInitialLandmarks]];
            dijk.findSSSPDistances(graph, randomVertex, landmarkDistances, pqueue);

            for (auto j = 0; j < objectNodes.size(); j++) {
                //// get vertex weight
                vertexWeight = landmarkDistances[objectNodes[j]] - getLowerBound(randomVertex, objectNodes[j]);
                std::vector<NodeID> readyToPush;

                adjListStart = graph.getEdgeListStartIndex(objectNodes[j]);
                nextAdjListStart = graph.getEdgeListSize(objectNodes[j]);

                // check if a child in the Shortest Path Tree
                for (int k = adjListStart; k < nextAdjListStart; ++k) {
                    if (landmarkDistances[graph.edges[k].first] == landmarkDistances[objectNodes[j]] + graph.edges[k].second) {
                        readyToPush.push_back(graph.edges[k].first);
                    }
                }
                // Add node - a tuple of vertex children and weight - to the tree
                spTree[objectNodes[j]] = std::make_tuple(readyToPush, vertexWeight, 0ul);
            }

            // recursively calculate sizes of each node v
            // (size is defined as a sum of weights in a subtree of the spTree with node v as a root)
            // if a node is a landmark - size = 0
            calculateSizes(randomVertex, spTree, landmarks);

            // Calculate a path of max size - on each node go to node with highest size, repeat until leaf.
            landmarks.push_back(getMaxLeaf(randomVertex, spTree));

            // perform normal landmark creation
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
            }
        }
        delete pqueue;
        objectList.setDistances(objectDistances, objectNodes.size());
        return;
    }
    else if (landmarkType == LANDMARK_TYPE::AVOID_PEQUE_URATA_IRYO) {
        SetGenerator sg;
        //// Number to divide the number of total landmarks
        int divideBy = 10;
        //// get the rounded up number of divided landmarks
        int numInitialLandmarks = std::ceil(numLandmarks / divideBy);
        std::vector<NodeID> randomSet = sg.generateRandomSampleSet(objectNodes.size(), numLandmarks);
        //// init first a certain % of landmark starting points
        std::vector<NodeID> initialLandmarkVertices(randomSet.begin(), randomSet.begin() + numInitialLandmarks);
        //// init random vertices from which we want to run avoid in every iteration to find a landmark candidate
        std::vector<NodeID> randomVertices(randomSet.begin() + numInitialLandmarks, randomSet.end());

        landmarks.clear();
        landmarks.reserve(randomSet.size());

        for (auto vert : initialLandmarkVertices) {
            landmarks.push_back(objectNodes[vert]);
        }

        //// init the first landmarks - de facto the random objects approach
        std::size_t startingIndex = initialLandmarkVertices.size();
        for (std::size_t i = 0; i < initialLandmarkVertices.size(); ++i) {
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
            }
        }

        //// Shortest path tree as a vector of tuples of children ids, weights, and sizes
        std::vector<std::tuple<std::vector<NodeID>, EdgeWeight, unsigned long, unsigned>> spTree(numNodes);
        int adjListStart, nextAdjListStart;
        NodeID randomVertex;
        EdgeWeight vertexWeight;

        for (std::size_t i = startingIndex; i < numLandmarks; ++i) {
            pqueue->clear();
            //// randomVertex should be indexed from 0
            randomVertex = objectNodes[randomVertices[i - numInitialLandmarks]];
            dijk.findSSSPDistances(graph, randomVertex, landmarkDistances, pqueue);

            for (auto j = 0; j < objectNodes.size(); j++) {
                //// get vertex weight
                vertexWeight = landmarkDistances[objectNodes[j]] - getLowerBound(randomVertex, objectNodes[j]);
                std::vector<NodeID> readyToPush;

                adjListStart = graph.getEdgeListStartIndex(objectNodes[j]);
                nextAdjListStart = graph.getEdgeListSize(objectNodes[j]);

                // check if a child in the Shortest Path Tree
                for (int k = adjListStart; k < nextAdjListStart; ++k) {
                    if (landmarkDistances[graph.edges[k].first] == landmarkDistances[objectNodes[j]] + graph.edges[k].second) {
                        readyToPush.push_back(graph.edges[k].first);
                    }
                }
                // Add node - a tuple of vertex children and weight - to the tree
                spTree[objectNodes[j]] = std::make_tuple(readyToPush, vertexWeight, 0ul, 0u);
            }

            // recursively calculate sizes of each node v
            // (size is defined as a sum of weights in a subtree of the spTree with node v as a root)
            // if a node is a landmark - size = 0
            calculateSizes(randomVertex, spTree, landmarks);

            // Calculate a path of max size - on each node go to node with highest size, repeat until leaf.
            landmarks.push_back(getMaxLeaf(randomVertex, spTree));

            // perform normal landmark creation
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
            }
        }
        delete pqueue;
        objectList.setDistances(objectDistances, objectNodes.size());
        return;
    }
    else if (landmarkType == LANDMARK_TYPE::ADVANCED_AVOID) {
        SetGenerator sg;
        //// Number to divide the number of total landmarks
        int divideBy = 10;
        //// get the rounded up number of divided landmarks
        int numInitialLandmarks = std::ceil(numLandmarks / divideBy);
        std::vector<NodeID> randomSet = sg.generateRandomSampleSet(objectNodes.size(), numLandmarks);
        //// init first a certain % of landmark starting points
        std::vector<NodeID> initialLandmarkVertices(randomSet.begin(), randomSet.begin() + numInitialLandmarks);
        //// init random vertices from which we want to run avoid in every iteration to find a landmark candidate
        std::vector<NodeID> randomVertices(randomSet.begin() + numInitialLandmarks, randomSet.end());
        //// num repeats
        unsigned numRepeats = 1;
        //// num of discarded landmarks per repeat
        unsigned numDiscarded = 6;

        landmarks.clear();
        landmarks.reserve(randomSet.size());

        for (auto vert : initialLandmarkVertices) {
            landmarks.push_back(objectNodes[vert]);
        }

        std::size_t startingIndex = initialLandmarkVertices.size();
        for (std::size_t i = 0; i < initialLandmarkVertices.size(); ++i) {
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
            }
        }

        std::vector<std::tuple<std::vector<NodeID>, EdgeWeight, unsigned long, unsigned>> spTree(numNodes);
        int adjListStart, nextAdjListStart;
        NodeID randomVertex;
        EdgeWeight vertexWeight;

        for (std::size_t i = startingIndex; i < numLandmarks; ++i) {
            pqueue->clear();
            randomVertex = objectNodes[randomVertices[i - numInitialLandmarks]];
            dijk.findSSSPDistances(graph, randomVertex, landmarkDistances, pqueue);
            for (auto j = 0; j < objectNodes.size(); j++) {
                vertexWeight = landmarkDistances[objectNodes[j]] - getLowerBound(randomVertex, objectNodes[j]);
                std::vector<NodeID> readyToPush;
                adjListStart = graph.getEdgeListStartIndex(objectNodes[j]);
                nextAdjListStart = graph.getEdgeListSize(objectNodes[j]);
                for (int k = adjListStart; k < nextAdjListStart; ++k) {
                    if (landmarkDistances[graph.edges[k].first] == landmarkDistances[objectNodes[j]] + graph.edges[k].second) {
                        readyToPush.push_back(graph.edges[k].first);
                    }
                }
                spTree[objectNodes[j]] = std::make_tuple(readyToPush, vertexWeight, 0ul, 0u);
            }

            calculateSizes(randomVertex, spTree, landmarks);
            landmarks.push_back(getMaxLeaf(randomVertex, spTree));
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
            }
        }
        for (int r = 0; r < numRepeats; r++) {
            std::vector <unsigned> replaced = sg.generateRandomSampleSet(numLandmarks, numDiscarded);
            randomVertices = sg.generateRandomSampleSet(objectNodes.size(), numDiscarded);
            std::vector < std::tuple < std::vector < NodeID > , EdgeWeight, unsigned long, unsigned >> spTree(numNodes);
            unsigned adjListStart, nextAdjListStart;
            NodeID randomVertex;
            EdgeWeight vertexWeight;
            unsigned iterator = 0;

            for (std::size_t i: replaced) {
                pqueue->clear();
                randomVertex = objectNodes[randomVertices[iterator]];
                dijk.findSSSPDistances(graph, randomVertex, landmarkDistances, pqueue);
                for (auto j = 0; j < objectNodes.size(); j++) {
                    vertexWeight = landmarkDistances[objectNodes[j]] - getLowerBound(randomVertex, objectNodes[j]);
                    std::vector <NodeID> readyToPush;
                    adjListStart = graph.getEdgeListStartIndex(objectNodes[j]);
                    nextAdjListStart = graph.getEdgeListSize(objectNodes[j]);

                    for (int k = adjListStart; k < nextAdjListStart; ++k) {
                        if (landmarkDistances[graph.edges[k].first] ==
                            landmarkDistances[objectNodes[j]] + graph.edges[k].second) {
                            readyToPush.push_back(graph.edges[k].first);
                        }
                    }
                    spTree[objectNodes[j]] = std::make_tuple(readyToPush, vertexWeight, 0ul, 0u);
                }

                calculateSizes(randomVertex, spTree, landmarks);
                landmarks[i] = getMaxLeaf(randomVertex, spTree);
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
                }
                iterator++;
            }
        }
        delete pqueue;
        objectList.setDistances(objectDistances, objectNodes.size());
        return;
    } else {
        std::cerr << "Unknown landmark type" << std::endl;
        std::exit(1);
    }

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

void ALT::generateMinDistLandmarks(Graph &graph, unsigned int maxNumLandmarks)
{
    landmarks.clear();
    landmarks.reserve(maxNumLandmarks);
    // Allocated space for distance from each landmark to each graph vertex
    vertexFromLandmarkDistances.resize(maxNumLandmarks * numNodes);
    std::vector<std::vector<unsigned>> landmarksPathLengths(maxNumLandmarks, std::vector<unsigned>((unsigned)graph.getNumNodes(), 0));
    std::vector<unsigned> landmarksMaxPaths(maxNumLandmarks, 1);

    // Use Dijkstra's to populate above vector for each landmark
    DijkstraSearch dijk;
    std::vector<EdgeWeight> landmarkDistances(numNodes, 0);
    std::vector<EdgeWeight> nodesVisited(numNodes, 0);
    BinaryMinHeap<EdgeWeight, NodeData> pqueue;

    unsigned numberOfTries = 4096;
    double threshold = parameters.threshold;
    unsigned currentNumLandmarks = 0;

    for(unsigned j = 0; j < numberOfTries; ++j) {
        NodeID node = rand() % graph.getNumNodes();
        auto landmarkNodesRatio = closestLandmarkNodesRatio(node, landmarksPathLengths, landmarksMaxPaths, currentNumLandmarks);
        if(landmarkNodesRatio > threshold) {
//            std::cout << "Creating landmark " << currentNumLandmarks << "... Dist ratio:" << landmarkNodesRatio << ", try number: " << j << std::endl;
            pqueue.clear();
            landmarks.push_back(node);
            dijk.findSSSPDistances(graph, landmarks[currentNumLandmarks], landmarkDistances,
                                   landmarksPathLengths[currentNumLandmarks], nodesVisited, &pqueue);

            for (std::size_t k = 0; k < numNodes; ++k) {
                vertexFromLandmarkDistances[k * maxNumLandmarks + currentNumLandmarks] = landmarkDistances[k];
                if(landmarksMaxPaths[currentNumLandmarks] < landmarksPathLengths[currentNumLandmarks][k]) {
                    landmarksMaxPaths[currentNumLandmarks] = landmarksPathLengths[currentNumLandmarks][k];
                }
            }
            currentNumLandmarks++;
            if(currentNumLandmarks >= maxNumLandmarks) {
                break;
            }
        }
    }

//    this->numLandmarks = currentNumLandmarks;
    std::cout << "Created " << currentNumLandmarks << "/" << maxNumLandmarks << std::endl;
}

double ALT::closestLandmarkNodesRatio(NodeID node, std::vector<std::vector<unsigned>>& landmarksPathLengths,
                                      std::vector<unsigned>& landmarksMaxPaths, unsigned numLandmarks)
{
    double bestRatio = 1;
    for (unsigned i = 0; i < numLandmarks; ++i) {
        auto ratio = landmarksPathLengths[i][node] / (double) landmarksMaxPaths[i];
        if (ratio < bestRatio) {
            bestRatio = ratio;
        }
    }
    return bestRatio;
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

EdgeWeight ALT::getLowerBound(NodeID s, NodeID t, std::vector<unsigned int> &landmarkIndexes)
{
    EdgeWeight globalLB = 0, currentLB;
    for (auto i : landmarkIndexes) {
        currentLB = std::abs(
                vertexFromLandmarkDistances[s * numLandmarks + i] - vertexFromLandmarkDistances[t * numLandmarks + i]);
        if (currentLB > globalLB) {
            globalLB = currentLB;
        }
    }
    return globalLB;
}

EdgeWeight ALT::getLowestLowerBound(NodeID s, std::vector<NodeID> &targets)
{
    EdgeWeight lowestLowerBound = getLowerBound(s, targets.front());
    for(int i = 1; i < targets.size(); ++i) {
        EdgeWeight lb = getLowerBound(s, targets[i]);
        if(lb < lowestLowerBound) {
            lowestLowerBound = lb;
        }
    }
    return lowestLowerBound;
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

                    sourceToAdjNodeDist = minDist + getEdgeWeight(graph, i);
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

std::vector<std::pair<NodeID, EdgeWeight>> ALT::findShortestPathDistances(Graph &graph, NodeID source, std::vector<NodeID>& targets)
{
    edgesAccessed.clear();

    std::vector<std::pair<NodeID, EdgeWeight>> results;
    results.reserve(targets.size());
    BinaryMinHeap<EdgeWeight, NodeDistancePair> pqueue;
    std::vector<bool> isNodeSettled(graph.getNumNodes(), false);

//     EdgeWeight adjNodeWgt;
    EdgeWeight minDist, sourceToAdjNodeDist;
    NodeID minDistNodeID, adjNode;
    int adjListStart, adjListSize;

    // Initialize priority queue with source node
    EdgeWeight minSourceTargetDist = getLowestLowerBound(source, targets);
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

            if (std::find(targets.begin(), targets.end(), minDistNodeID) != targets.end()) {
                // If the minimum is the target we have finished searching
                results.emplace_back(minDistNodeID, minDist);
                if(results.size() >= targets.size()) {
                    break;
                }
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
                    minSourceTargetDist = sourceToAdjNodeDist + getLowestLowerBound(adjNode, targets);
                    pqueue.insert(NodeDistancePair(adjNode, sourceToAdjNodeDist), minSourceTargetDist);
                }
            }
        }
    }

    edgesAccessedCount += edgesAccessed.size();

    if(results.size() < targets.size()) {
        std::cout << "Did not break early" << std::endl;
    }
    return results;
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

std::vector<unsigned> ALT::selectBestLandmarks(NodeID s, NodeID t)
{
    const unsigned numberOfSelectedLandmarks = 10;
    std::vector<unsigned> bestLandmarks(numberOfSelectedLandmarks, 0);
    std::vector<unsigned> bestBounds(numberOfSelectedLandmarks, 0);


    EdgeWeight currentLB;
    for (std::size_t i = 0; i < landmarks.size(); ++i) {
        currentLB = std::abs(
                (int) nodeFromLandmarkDistance(i, s) - (int) nodeFromLandmarkDistance(i, t));
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




