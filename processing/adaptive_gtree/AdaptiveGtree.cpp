/*
 *
 * This file is a modified copy of a file in Road Network kNN Experimental Evaluation.
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

#include "AdaptiveGtree.h"
#include "AdaptiveGtreeNode.h"
#include "../DynamicGraph.h"

#include "../DijkstraSearch.h"
#include "../../utility/StopWatch.h"
#include "../../utility/utility.h"
#include "DistanceMatrixBuilder.h"
#include "../../utils/Logger.h"

#include <assert.h>
#include <iostream>

AdaptiveGtree::AdaptiveGtree(std::string networkName, int numNodes, int numEdges, int fanout, std::size_t maxLeafSize) :
        networkName(networkName), numNodes(numNodes), numEdges(numEdges), fanout(fanout), maxLeafSize(maxLeafSize)
{
    this->nodeIDLeafVerticesVecIdx.assign(this->numNodes, -1);
    this->nodeIDLeafBordersVecIdx.assign(this->numNodes, -1);
    this->edgeInLeafSubgraph.assign(this->numEdges, true);
    this->nodeLeafIdxs.assign(this->numNodes, -1);
}

AdaptiveGtree::AdaptiveGtree()
{
}

std::string AdaptiveGtree::getNetworkName()
{
    return this->networkName;
}

int AdaptiveGtree::getNumEdges()
{
    return this->numEdges;
}

int AdaptiveGtree::getNumNodes()
{
    return this->numNodes;
}

int AdaptiveGtree::getFanout()
{
    return this->fanout;
}

int AdaptiveGtree::getTreeSize()
{
    return this->treeNodes.size();
}

std::size_t AdaptiveGtree::getMaxLeafSize()
{
    return this->maxLeafSize;
}

int AdaptiveGtree::getLeafIndex(NodeID nodeID)
{
    return this->nodeLeafIdxs[nodeID];
}

int AdaptiveGtree::getParentIndex(int treeIdx)
{
    return this->treeNodes[treeIdx].getParentIdx();
}

void AdaptiveGtree::buildGtree(Graph &graph)
{

    StopWatch sw;
    sw.start();
    this->buildTreeHierarchy(graph);
    sw.stop();
    std::cout << " [AdaptiveGtree] buildTreeHierarchy " << sw.getTimeMs() << std::endl;

    sw.reset();
    sw.start();
    this->computeDistanceMatrix(graph);
    sw.stop();
    std::cout << " [AdaptiveGtree] computeDistanceMatrix " << sw.getTimeMs() << std::endl;

    this->initialiseGtreeQueryStructure();

}

void AdaptiveGtree::buildTreeHierarchy(Graph &graph)
{
    METISIdxToNodeID.resize(this->numNodes);
    METISWrapper metis(this->numNodes, this->numEdges, this->fanout);

    std::unordered_set<NodeID> subgraph = graph.getNodesIDsUset();
    this->addNode(ROOT_PARENT_INDEX, subgraph, graph, metis);
}

void
AdaptiveGtree::addNode(int parentIdx, std::unordered_set<NodeID> &subgraph, Graph &originalGraph, METISWrapper &metis)
{
    // Create Gtree node and add to tree
    int treeIdx = this->treeNodes.size();
    int childvecIdx;
    AdaptiveGtreeNode treeNode(treeIdx, parentIdx, subgraph.size());
    this->treeNodes.push_back(treeNode);

    if (parentIdx != ROOT_PARENT_INDEX) {
        // Add this Gtree node as child of parent (if not root)
        childvecIdx = this->treeNodes[parentIdx].addChild(treeIdx);
        this->treeNodes[treeIdx].setParentChildIdx(childvecIdx);

        // Add parent's Gtree path to this nodes Gtree path then add parent
        this->treeNodes[treeIdx].addGtreePathNodes(this->treeNodes[parentIdx].gtreePath);
    }
    this->treeNodes[treeIdx].addGtreePathNode(treeIdx);

    // Find and add children of this node
    if (subgraph.size() > this->maxLeafSize) {
        this->addChildren(treeIdx, subgraph, originalGraph, metis);
    } else {
        // If this has less than tau graph node then we stop partitions
        this->treeNodes[treeIdx].setLeafNode();
        this->leafIdxs.push_back(treeIdx);
    }

    // Determine and add borders for this Gtree node
    if (parentIdx != ROOT_PARENT_INDEX) {
        // We mark the starting index of this tree nodes borders
        // in the parent's childBordersVec - we can do this as
        // we are about find all these borders and also add 
        // consecutively to the parent's childBordersVec
        this->treeNodes[parentIdx].addChildOffsetInChildBorderVec();
        int adjListStart, adjListSize, leafVerticesVecIdx, leafBordersVecIdx;
        bool isBorder;

        for (NodeID node: subgraph) {
            // If this node is a leaf node, we also need to map 
            // all it's graph nodes to they're Gtree leaf index
            if (this->treeNodes[treeIdx].isLeafNode()) {
                // Add each node as a vertice if it is (this
                // will include borders)
                leafVerticesVecIdx = this->treeNodes[treeIdx].addLeafVertex(node);
                this->setLeafVerticesVecIdx(node, leafVerticesVecIdx);
                this->nodeLeafIdxs[node] = treeIdx;
            }

            adjListStart = originalGraph.getEdgeListStartIndex(node);
            adjListSize = originalGraph.getEdgeListSize(node);
            isBorder = false;
            for (int i = adjListStart; i < adjListSize; ++i) {
                if (subgraph.find(originalGraph.edges[i].first) == subgraph.end()) {
                    // This means we have found a neighbour of that outside this subgraph i.e.
                    // this is node is a border so we add to list of borders of this tree node
                    // and the set of child borders for the parent
                    if (!isBorder) {
                        leafBordersVecIdx = this->treeNodes[treeIdx].addBorder(node);
                        if (this->treeNodes[treeIdx].isLeafNode()) {
                            this->setLeafBordersVecIdx(node, leafBordersVecIdx);
                        }
                        this->treeNodes[parentIdx].addChildBorder(node);
                        isBorder = true; // So that we don't add it again
                    }
                    if (this->treeNodes[treeIdx].isLeafNode()) {
                        // Since this edge leads to a node that is not in the subgraph
                        this->setEdgeNotInSubgraph(i);
                    }
                }
            }
        }

    }

}

void AdaptiveGtree::addChildren(int parentTreeIdx, std::unordered_set<NodeID> &parentGraph, Graph &originalGraph,
                                METISWrapper &metis)
{
    // Assuming original graph is undirected graph (if not we must make undirected
    // in order for METIS to function)

    idx_t n = parentGraph.size();

    metis.populateMETISArrays(originalGraph, parentGraph, this->METISIdxToNodeID, true);

    metis.partitionSubgraph(n);

    // Create sets of subgraphs from partitioned parent graph
    // and add them to Gtree (and this will recurse)
    std::vector<std::unordered_set<NodeID>> childGraphs(this->fanout);

    for (int i = 0; i < n; ++i) {
        //assert (nodePartitions[i] <= this->fanout && "Invalid partition assigned to graph node, too many partitions");
        // Create child graphs using original NodeID not the METIS idx
        childGraphs[metis.parts[i]].insert(this->METISIdxToNodeID[i]);
    }

    // Also add children of this node to Gtree
    for (std::size_t i = 0; i < childGraphs.size(); ++i) {
        this->addNode(parentTreeIdx, childGraphs[i], originalGraph, metis);
        // Note: We release child graph at this point as it is not need again
        // and this would improve overall memory usage (it won't affect vector size)
        utility::releaseSTLCollection(childGraphs[i]);
    }
}

void AdaptiveGtree::computeDistanceMatrix(Graph &graph)
{
    std::vector<std::vector<int>> treeLevelIdxs = this->getTreeNodesByLevel();

    DijkstraSearch dijkstra;
    BinaryMinHeap<EdgeWeight, NodeID> *pqueue = new BinaryMinHeap<EdgeWeight, NodeID>();
    int currentIdx;
    DynamicGraph tempGraph(graph);
    std::unordered_set<NodeID> *targets;
    std::vector<NodeID> adjNodes;
    std::vector<EdgeWeight> adjNodeWgts;
    std::vector<NodeID> *sourcesVec, *targetsVec;

    for (int i = treeLevelIdxs.size() - 1; i >= 0; --i) {
        // Clear memory in unordered_map or it will continue to grow
        // Note: According to the paper, total number of borders at each level should be O(n)
        // so if we clear this map for each level then we should it's total size should be O(n)

        for (std::size_t j = 0; j < treeLevelIdxs[i].size(); ++j) {
            currentIdx = treeLevelIdxs[i][j];
            if (this->treeNodes[currentIdx].isLeafNode()) {
                // In a leaf we find distances from border to all leaf vertices
                sourcesVec = &this->treeNodes[currentIdx].getBorders();
                targets = &this->treeNodes[currentIdx].getLeafVerticesUset();
                targetsVec = &this->treeNodes[currentIdx].getLeafVertices();
            } else {
                // In a non-leaf we get distances from all child borders to all other child borders
                // Note: It's possible some of these distances have already been computed
                sourcesVec = &this->treeNodes[currentIdx].getChildBorders();
                targets = &this->treeNodes[currentIdx].getChildBordersUset();
                targetsVec = &this->treeNodes[currentIdx].getChildBorders();
            }

            std::unordered_map<NodeID, EdgeWeight> siblingBorderDistances;
            siblingBorderDistances.reserve(targetsVec->size());
            //Using single std::unordered_map
            int rowLength = targetsVec->size();
            this->treeNodes[currentIdx].matrixRowLength = rowLength;
            this->treeNodes[currentIdx].distanceMatrix.init(targetsVec->size(), sourcesVec->size());

            if (this->treeNodes[currentIdx].isLeafNode()) {
                // distances for leaves are calculated adaptively
                for (std::size_t i = 0; i < sourcesVec->size(); ++i) {
                    for (std::size_t j = 0; j < targetsVec->size(); ++j) {
                        this->treeNodes[currentIdx].distanceMatrix.pushBackNotAssigned();
                    }
                }
            } else {
                // distances for internal nodes are precalculated at the moment
                for (std::size_t i = 0; i < sourcesVec->size(); ++i) {
//                    pqueue->clear();
//                    dijkstra.findSSMTDistances(graph, (*sourcesVec)[i], (*targets), siblingBorderDistances, pqueue);
                    for (std::size_t j = 0; j < targetsVec->size(); ++j) {
//                        this->treeNodes[currentIdx].distanceMatrix.push_back(siblingBorderDistances[(*targetsVec)[j]]);
                        this->treeNodes[currentIdx].distanceMatrix.pushBackNotAssigned();
                    }
                }
            }
//            if (this->treeNodes[currentIdx].isLeafNode()) {
            // If it is a leaf node we can search using original Graph
            // who's data structure is faster than DynamicGraph

//            } else {
//                for (std::size_t i = 0; i < sourcesVec->size(); ++i) {
//                    pqueue->clear();
//                    dijkstra.findSSMTDistances(tempGraph,(*sourcesVec)[i],(*targets),siblingBorderDistances,pqueue);
//                    for (std::size_t j = 0; j < targetsVec->size(); ++j) {
//                        this->treeNodes[currentIdx].distanceMatrix.push_back(siblingBorderDistances[(*targetsVec)[j]]);
//                    }
//                }
//            }

            // All future searches will be on this nodes border set (using closure property in paper)
            // because these will be the parent nodes child borders-> Therefore if we need only remove
            // unnecessary edges from these nodes (unimportant nodes will become disconnected)
            //sources = this->treeNodes[currentIdx].getBordersUset();
//            sourcesVec = &this->treeNodes[currentIdx].getBorders();
//
//            NodeID border;
//            int sourceIdx, targetIdx;
//            for (std::size_t i = 0; i < sourcesVec->size(); ++i) {
//                border = (*sourcesVec)[i];
//                if (this->treeNodes[currentIdx].isLeafNode()) {
//                    sourceIdx = i;
//                } else {
//                    sourceIdx = this->treeNodes[currentIdx].getBorderIdxInChildBorderVec(i);
//                }
//
//                // Preserve edges to outside this gtree node (i.e. subgraph)
//                adjNodes.clear();
//                adjNodeWgts.clear();
//                for (std::size_t i = 0; i < tempGraph.nodes[border].adjNodes.size(); ++i) {
//                    if (targets->find(tempGraph.nodes[border].adjNodes[i]) == targets->end()) {
//                        // This check whether the adj node is within the current gtree node
//                        // if it is we do not need to preserve the edge
//                        adjNodes.push_back(tempGraph.nodes[border].adjNodes[i]);
//                        adjNodeWgts.push_back(tempGraph.nodes[border].adjNodeWgts[i]);
//                    }
//                }
//                // Note: That this will make the graph disconnected but this doesn't
//                // matter as removing disconnected node won't change search results and
//                // we are only disconnecting nodes that are not borders of the subgraph
//                // (i.e. they will not be needed again at higher levels)
//
//                tempGraph.nodes[border].adjNodes = std::move(adjNodes);
//                tempGraph.nodes[border].adjNodeWgts = std::move(adjNodeWgts);
//                for (std::size_t j = 0; j < sourcesVec->size(); ++j) {
//                    targetIdx = this->treeNodes[currentIdx].getBorderIdxInChildBorderVec(j); // We will return dist matrix idx whether leaf or not
//                    if (border != (*sourcesVec)[j]) {
//                        tempGraph.insertImaginaryNonInvertibleEdge(border,(*sourcesVec)[j],this->treeNodes[currentIdx].distanceMatrix.get(sourceIdx, targetIdx));
//                    }
//                }
//            }
        }
    }

    delete pqueue;
}

void AdaptiveGtree::initialiseGtreeQueryStructure()
{
    // Allocate Memory for Gtree Query
    this->sourceToTreeNodeBorderDist.resize(this->treeNodes.size());
    for (size_t i = 0; i < this->treeNodes.size(); ++i) {
        this->sourceToTreeNodeBorderDist[i].resize(this->treeNodes[i].bordersVec.size());
    }
}

void AdaptiveGtree::printLevels()
{
    std::vector<std::vector<int>> treeNodeLevel = this->getTreeNodesByLevel();

    for (int i = treeNodeLevel.size() - 1; i >= 0; --i) {
        std::cout << "Level " << i << ": " << treeNodeLevel[i].size() << std::endl;
    }
}

int AdaptiveGtree::getNumLevels()
{
    return this->getTreeNodesByLevel().size();
}

int AdaptiveGtree::getNumBorders()
{
    int numBorders = 0;
    for (std::size_t i = 0; i < this->treeNodes.size(); ++i) {
        numBorders += this->treeNodes[i].getNumBorders();
    }
    return numBorders;
}

int AdaptiveGtree::getBorderToBorderRelationships()
{
    int numB2BRelationships = 0;
    for (std::size_t i = 0; i < this->treeNodes.size(); ++i) {
        numB2BRelationships += this->treeNodes[i].getNumBorders() * this->treeNodes[i].getNumBorders();
    }
    return numB2BRelationships;
}

int AdaptiveGtree::getAvgPathCost(int treeIdx)
{
    int avgPathCost = 0;
    if (this->treeNodes[treeIdx].isLeafNode()) {
        avgPathCost = this->treeNodes[treeIdx].getNumBorders();
    } else {
        //assert(this->treeNodes[treeIdx].children.size() > 0); Non-leaf nodes must have at least one child
        int totalChildPathCosts = 0, totalParentChildCost = 0;
        for (std::size_t i = 0; i < this->treeNodes[treeIdx].children.size(); ++i) {
            totalParentChildCost += this->treeNodes[treeIdx].getNumBorders() *
                                    this->treeNodes[this->treeNodes[treeIdx].children[i]].getNumBorders();
            totalChildPathCosts += this->getAvgPathCost(this->treeNodes[treeIdx].children[i]);
        }
        avgPathCost += totalParentChildCost / this->treeNodes[treeIdx].children.size();
        avgPathCost += totalChildPathCosts / this->treeNodes[treeIdx].children.size();
    }
    return avgPathCost;
}

std::vector<std::vector<int> > AdaptiveGtree::getTreeNodesByLevel()
{
    std::vector<std::vector<int>> treeNodeLevel;
    std::vector<int> currentLevel, nextLevel;
    nextLevel.push_back(0);

    while (nextLevel.size() != 0) {
        treeNodeLevel.push_back(nextLevel);
        currentLevel.swap(nextLevel);
        nextLevel.clear();
        for (int i: currentLevel) {
            for (std::size_t j = 0; j < this->treeNodes[i].children.size(); j++) {
                nextLevel.push_back(this->treeNodes[i].children[j]);
            }
        }
    }

    return treeNodeLevel;
}

void AdaptiveGtree::printNode(int treeIdx)
{
    this->treeNodes[treeIdx].printNode();
}

EdgeWeight AdaptiveGtree::getShortestPathDistance(Graph &graph, NodeID u, NodeID v)
{
#if defined(COLLECT_STATISTICS)
    this->stats.clear();
    this->stats.initialiseStatistic("computations_materialized",0);
    this->stats.initialiseStatistic("computations_executed",0);
    this->stats.initialiseStatistic("computations_total",0);
#endif
    EdgeWeight spDist = 0;
    int uLeaf = this->getLeafIndex(u);
    int vLeaf = this->getLeafIndex(v);

    if (uLeaf == vLeaf) {
        // This mean they are both in the same node
        spDist = this->SPDistLeaf(u, v, uLeaf, graph);
    } else {
        int LCAIdx = 0;
        std::vector<int> &uPathFromRoot = this->treeNodes[uLeaf].getGtreePathFromRoot();
        std::vector<int> &vPathFromRoot = this->treeNodes[vLeaf].getGtreePathFromRoot();

        // Since the two nodes are in different leaf nodes we can guarantee that
        // there is at least one more node (the parent of both) in the G-tree path
        // which we call the lowest common ancestor (LCA)

        // Search down Gtree from root until we find first ancestor that is different
        // this means the previous ancestor is the LCA and the indexes point its children
        unsigned int i, j;
        for (i = 0, j = 0; i < uPathFromRoot.size() && j < vPathFromRoot.size(); ++i, ++j) {
            if (uPathFromRoot[i] != vPathFromRoot[j]) {
                // When i = 0 and j = 0 it is referring to the root so in that
                // case uPathFromRoot[i] does equal vPathFromRoot[j]. This means
                // when they are equal i > 0, so i-1 is safe here
                LCAIdx = uPathFromRoot[i - 1];
                break;
            }
            // Note: We can guarantee that LCAIdx will be set here. The only situation
            // it would not be set is both u and v were in the same leaf, but we have
            // guaranteed that is not the case in the if/else statement
        }

        // Now search up G-tree (from source leaf) until we reach i and j, then we 
        // search down (to target leaf) computing the shortest path distance

        // From source to source leaf
        this->SPDistToSourceLeafNode(graph, u, uLeaf);

        // From source leaf to first child of LCA
        int x = i; // This is safe, depth of tree is O(log(n)) and n is at most 24 million in US dataset
        for (int k = uPathFromRoot.size() - 1; k > x; --k) {
            // Since k > x and x is at worst 0, k-1 is safe here
            this->SPDistToParentNode(graph, uPathFromRoot[k], uPathFromRoot[k - 1], false);
        }

        // From first child of LCA to second child of LCA
        this->SPDistToSiblingNode(graph, uPathFromRoot[i], vPathFromRoot[j], LCAIdx, false);

        // From second child of LCA to target leaf
        for (std::size_t k = j; k < vPathFromRoot.size() - 1; ++k) {
            // Note the size()-1 in the above condition
            this->SPDistToChildNode(graph, vPathFromRoot[k + 1], vPathFromRoot[k], false);
        }

        // We assume target has not been visited
        spDist = this->SPDistToLeafTarget(graph, v, vLeaf);
    }

    return spDist;
}

EdgeWeight AdaptiveGtree::SPDistLeaf(NodeID u, NodeID v, int leafNode, Graph &graph)
{
    // Assume u and v are in the same leaf node
    EdgeWeight borderToBorderDist, spDist = 0;

    spDist = this->DijkstraDist(u, v, leafNode, graph);
    borderToBorderDist = this->BorderDist(u, v, leafNode);
    if (borderToBorderDist < spDist) {
        spDist = borderToBorderDist;
    }

    return spDist;
}

std::unordered_map<NodeID, EdgeWeight>
AdaptiveGtree::DijkstraDistMultiTarget(NodeID u, std::unordered_set<NodeID> &targets, int leafNode, Graph &graph)
{
    // Assume u and all targets are in the same leaf node and number of targets > 0

    DijkstraSearch dijkstra;
    std::unordered_map<NodeID, EdgeWeight> targetDistances;

    // This is equivalent of DijkDist function in paper, except we optimise by 
    // doing multi-target search in case there are many objects in source leaf
    dijkstra.findSSMTDistancesSubgraph(graph, u, targets, targetDistances, this->edgeInLeafSubgraph);

    return targetDistances;
}

//EdgeWeight AdaptiveGtree::SPDist(NodeID u, NodeID v, std::vector<int> &gtreePath, int firstLCAChild)
//{
//    // Assume gtreePath size is 2 or more (i.e. u and v are not in same leaf node)
//    // Assume that no results have been materialised (this for shortest path query)
//    // Note: Path cannot be size 1 since that make one parent of u and v a leaf node
//    // which cannot be possible as a leaf node has no children
//
//    EdgeWeight spDist = 0;
//
//    // Distance from query node to source leaf borders
//    std::size_t i = 0;
//    // Commented out because it doesn't compile, graph is not available here
////    this->SPDistToSourceLeafNode(graph, u,gtreePath[i]);
//
//    // Distance up the G-tree hierarchy from source leaf
//    for (; gtreePath[i] != firstLCAChild; ++i) {
//        this->SPDistToParentNode(gtreePath[i], gtreePath[i + 1], false);
//    }
//
//    // Distance between siblings of LCA
//    int LCAIdx = this->treeNodes[gtreePath[i]].getParentIdx();
//    this->SPDistToSiblingNode(gtreePath[i], gtreePath[i + 1], LCAIdx, false);
//    ++i;
//
//    // Distance down G-tree hierarchy to target leaf
//    for (; i + 1 < gtreePath.size(); ++i) {
//        this->SPDistToChildNode(gtreePath[i + 1], gtreePath[i], false);
//    }
//
//    // doesn't compile
////    spDist = this->SPDistToLeafTarget(graph, v,gtreePath[i]);
//
//    return spDist;
//}

EdgeWeight AdaptiveGtree::BorderDist(NodeID u, NodeID v, int leafNode)
{
    // Assume that u and v ARE in the same leaf node

    EdgeWeight borderDist, minDist = 0;

#if defined(GTREE_STL_HASH_TABLE_DIST_MATRIX) || defined(GTREE_GOOGLE_DENSEHASH_DIST_MATRIX)
    if (this->treeNodes[leafNode].bordersVec.size() > 0) {
        // Initialise the minDist as the distance from the source to the first
        // border plus the distance from that border to the target
        minDist = this->distanceMatrix[this->treeNodes[leafNode].bordersVec[0]][u] + 
            this->distanceMatrix[this->treeNodes[leafNode].bordersVec[0]][v];
        for (std::size_t i = 0; i < this->treeNodes[leafNode].bordersVec.size(); ++i) {
            for (std::size_t j = 0; j < this->treeNodes[leafNode].bordersVec.size(); ++j) {
                // ASSUMING UNDIRECTED GRAPH (I.E. BORDER TO LEAF VERTEX DISTANCE = LEAF VERTEX TO BORDER DISTANCE
                // THIS IS BECAUSE WE HAVEN'T CALCULATED LEAF VERTEX TO BORDER DISTANCES
                if (i != j) {
                    // BorderDist return the distance between two nodes using outside vertices
                    // if i == j then we are entering and leaving through the same border
                    // which won't be smaller than the Dijkstra dist so we ignore it
                    borderDist = this->distanceMatrix[this->treeNodes[leafNode].bordersVec[i]][u] + 
                        this->distanceMatrix[this->treeNodes[leafNode].bordersVec[i]][this->treeNodes[leafNode].bordersVec[j]] + 
                        this->distanceMatrix[this->treeNodes[leafNode].bordersVec[j]][v];
                    if (borderDist < minDist) {
                        minDist = borderDist;
                    }
                }
            }
        }
    }
#else
    int uIdx = this->getIdxInLeafVerticesVec(u);
    int vIdx = this->getIdxInLeafVerticesVec(v);
    int intermediateBorderIdx;
    if (this->treeNodes[leafNode].bordersVec.size() > 0) {
        intermediateBorderIdx = this->treeNodes[leafNode].getBorderIdxInChildBorderVec(0);
        minDist = this->treeNodes[leafNode].distanceMatrix.atIndex(uIdx)
                  + this->treeNodes[leafNode].distanceMatrix.atIndex(intermediateBorderIdx)
                  + this->treeNodes[leafNode].distanceMatrix.atIndex(vIdx);
        for (std::size_t i = 0; i < this->treeNodes[leafNode].bordersVec.size(); ++i) {
            for (std::size_t j = 0; j < this->treeNodes[leafNode].bordersVec.size(); ++j) {
                // ASSUMING UNDIRECTED GRAPH (I.E. BORDER TO LEAF VERTEX DISTANCE = LEAF VERTEX TO BORDER DISTANCE
                // THIS IS BECAUSE WE HAVEN'T CALCULATED LEAF VERTEX TO BORDER DISTANCES
                if (i != j) {
                    // BorderDist return the distance between two nodes using outside vertices
                    // if i == j then we are entering and leaving through the same border
                    // which won't be smaller than the Dijkstra dist so we ignore it
                    intermediateBorderIdx = this->treeNodes[leafNode].getBorderIdxInChildBorderVec(j);
                    borderDist = this->treeNodes[leafNode].distanceMatrix.get(i, uIdx)
                                 + this->treeNodes[leafNode].distanceMatrix.get(i, intermediateBorderIdx)
                                 + this->treeNodes[leafNode].distanceMatrix.get(j, vIdx);
                    if (borderDist < minDist) {
                        minDist = borderDist;
                    }
                }
            }
        }
    }
#endif

#if defined(COLLECT_STATISTICS)
    this->stats.incrementStatistic("computations_executed",this->treeNodes[leafNode].bordersVec.size()*this->treeNodes[leafNode].bordersVec.size());
#endif

    return minDist;
}

EdgeWeight AdaptiveGtree::DijkstraDist(NodeID u, NodeID v, int leafNode, Graph &graph)
{
    // Assume that u and v ARE in the same leaf node

    DijkstraSearch dijk;

    return dijk.findShortestPathDistanceSubgraph(graph, u, v, this->edgeInLeafSubgraph);
}

// Note: The returned path does not include the LCA
std::vector<int> AdaptiveGtree::getGtreePath(NodeID u, NodeID v, int &firstLCAChild)
{
    // Note: If u and v are in the same leaf node then path will be 
    // size 0 and LCA will contain leaf node
    std::vector<int> path;

    int uLeaf = this->getLeafIndex(u);
    int vLeaf = this->getLeafIndex(v);

    std::vector<int> &uPathFromRoot = this->treeNodes[uLeaf].getGtreePathFromRoot();
    std::vector<int> &vPathFromRoot = this->treeNodes[vLeaf].getGtreePathFromRoot();

    // The only case where firstLCAChild is not set is when both nodes are in the same leaf
    // Note: This will be overwritten later if it needs to be
    firstLCAChild = uPathFromRoot.back();
    // Note: Using back() is safe as uPathFromRoot will have at least root in it

    // Search down Gtree from root until we find first ancestor that is different
    // this means the previous ancestor is the lowest common ancestor
    //assert(uPathFromRoot[0] == vPathFromRoot[0] == 0 && "First node in path from root is not root!")
    unsigned int i, j;
    for (i = 0, j = 0; i < uPathFromRoot.size() && j < vPathFromRoot.size(); ++i, ++j) {
        if (uPathFromRoot[i] != vPathFromRoot[j]) {
            firstLCAChild = uPathFromRoot[i];
            break;
        }
    }

    // Note: That we do not include LCA in return path
    int x = static_cast<int>(i); // This is OK, i is greater than 0 as above
    for (int k = uPathFromRoot.size() - 1; k >= x; --k) {
        // i represents the first non-common ancestor
        // so this will be the part of the path
        path.push_back(uPathFromRoot[k]);
    }

    // The path to v is in reverse order (we descend down)
    for (std::size_t k = j; k < vPathFromRoot.size(); ++k) {
        // j represents the first non-common ancestor
        // so this will be the part of the path
        path.push_back(vPathFromRoot[k]);
    }

    return path;

}

void AdaptiveGtree::getKNNs(OccurenceList &occList, unsigned int k, NodeID queryNode, std::vector<NodeID> &kNNs,
                            std::vector<EdgeWeight> &kNNDistances, Graph &graph)
{
#if defined(COLLECT_STATISTICS)
    this->stats.clear();
    this->stats.initialiseStatistic("computations_materialized",0);
    this->stats.initialiseStatistic("computations_executed",0);
    this->stats.initialiseStatistic("computations_total",0);
#endif
    // Note: We store both G-tree node indexes and road network NodeIDs in the
    // same priority queue. We  map the G-tree node indexes to a NodeID that 
    // is larger than the last node in the graph (and we check whenever 
    // a dequeue happens to determine what type min element is).
    BinaryMinHeap<EdgeWeight, NodeID> pqueue;

    int sourceLeaf = this->getLeafIndex(queryNode);
    int Tn, prevTn, treeIdx/*, firstLCAChild*/;
    EdgeWeight Tmin, dist, minKey, borderDist;
    NodeID minNode;

    Tn = sourceLeaf;
    Tmin = this->SPDistToSourceLeafNode(graph, queryNode, Tn);

    // Note: In order to use a priority queue with primitive types we
    // map the tree indexes as NodeID greater than the largest NodeID
    // for the current graph. Whenever we deque we can tell if the element
    // is a tree node or graph node by check if it's larger than the
    // number of nodes in the graph (since they are number 0 to n-1)
    NodeID firstMappedTreeNodeID = static_cast<NodeID>(this->numNodes);

    // If the source leaf node contains any object we must find them first
    if (occList.leafOccurenceList[sourceLeaf].size() > 0) {
#if defined(UNOPTIMISED_GTREE_LEAF_SEARCH)
        std::unordered_map<NodeID,EdgeWeight> leafObjectDistances = this->DijkstraDistMultiTarget(queryNode,occList.leafOccurenceSet[sourceLeaf],sourceLeaf,graph);
        for(NodeID leafObject: occList.leafOccurenceList[sourceLeaf]) {
            borderDist = this->BorderDist(queryNode,leafObject,sourceLeaf);
            if (borderDist < leafObjectDistances[leafObject]) {
                // If a shortest path can be found by leaving a border and re-entering source leaf through
                // another border then we use that distance because it is the actual shortest path distance
                pqueue.insert(leafObject,borderDist);
            } else {
                pqueue.insert(leafObject,leafObjectDistances[leafObject]);
            }
        }
#else
        if (this->getSourceLeafkNNsByINE(queryNode, k, occList.leafOccurenceSet[sourceLeaf], sourceLeaf, graph, kNNs,
                                         kNNDistances, pqueue)) {
            // If the in-leaf INE search was able to find definite k objects then we exist now
            return;
        }
#endif
    }

    while (pqueue.size() > 0 || Tn != 0) {
        // UpdateT
        if (pqueue.size() == 0) {
            prevTn = Tn;
            Tn = this->getParentIndex(Tn);

            if (Tn != 0) {
                // We don't set Tmin if Tn is the root because this represents
                // infinite instead - it's faster to check Tn != 0 than Tmin < max int
                Tmin = this->SPDistToParentNode(graph, prevTn, Tn);
#if defined(COLLECT_STATISTICS)
                this->stats.incrementStatistic("computations_materialized",this->getComputations(sourceLeaf,prevTn));
#endif
            }

            // Tn is a leaf only at start, but pqueue is size 1 at this point
            for (int childIdx: occList.nonLeafOccurenceList[Tn]) {
                if (childIdx != prevTn) {
                    // We only add children that we haven't already visited
                    dist = this->SPDistToSiblingNode(graph, prevTn, childIdx, Tn);
#if defined(COLLECT_STATISTICS)
                    this->stats.incrementStatistic("computations_materialized",this->getComputations(sourceLeaf,prevTn));
#endif
                    pqueue.insert(childIdx + this->numNodes, dist);
                }
            }

        }

        if (pqueue.size() > 0) {
            // This check is different to paper, but is necessary
            // as above UpdateT will not guarantee pqueue will have
            // size greater than 0 - in the case where the previous
            // Tn parent doesn't have any children with objects
            minKey = pqueue.getMinKey();
            minNode = pqueue.extractMinElement();
            if (Tn != 0 && minKey > Tmin) {
                // Tn == 0 is equivalent to Tmin being infinity
                // So using Tn != 0 we can avoid setting Tmin to infinity (i.e. max int value)
                // which was seen to slow down comparisons

                // UpdateT
                prevTn = Tn;
                Tn = this->getParentIndex(Tn);
                if (Tn != 0) {
                    // We don't set Tmin if Tn is the root because this represents
                    // infinite instead - it's faster to check Tn != 0 than Tmin < max int
                    Tmin = this->SPDistToParentNode(graph, prevTn, Tn);
#if defined(COLLECT_STATISTICS)
                    this->stats.incrementStatistic("computations_materialized",this->getComputations(sourceLeaf,prevTn));
#endif
                }

                // Tn is guaranteed to be a non-leaf here because we move to parent
                for (int childIdx: occList.nonLeafOccurenceList[Tn]) {
                    if (childIdx != prevTn) {
                        dist = this->SPDistToSiblingNode(graph, prevTn, childIdx, Tn);
#if defined(COLLECT_STATISTICS)
                        this->stats.incrementStatistic("computations_materialized",this->getComputations(sourceLeaf,prevTn));
#endif
                        pqueue.insert(childIdx + this->numNodes, dist);
                    }
                }

                pqueue.insert(minNode, minKey);
            } else if (minNode < firstMappedTreeNodeID) {
                // Min element is a graph node (i.e. object)
                kNNs.push_back(minNode);
                kNNDistances.push_back(minKey);
                if (kNNs.size() == k) {
                    // If this is the kth nearest neighbour object
                    // then we no longer need to iterate
                    break;
                }
            } else {
                treeIdx = minNode - this->numNodes;
                if (this->treeNodes[treeIdx].isLeafNode()) {
                    for (NodeID leafObject: occList.leafOccurenceList[treeIdx]) {
                        dist = this->SPDistToLeafTarget(graph, leafObject, treeIdx);
#if defined(COLLECT_STATISTICS)
                        this->stats.incrementStatistic("computations_materialized",this->getComputations(sourceLeaf,treeIdx));
#endif
                        pqueue.insert(leafObject, dist);
                    }
                } else {
                    for (int childIdx: occList.nonLeafOccurenceList[treeIdx]) {
                        dist = this->SPDistToChildNode(graph, childIdx, treeIdx);
#if defined(COLLECT_STATISTICS)
                        this->stats.incrementStatistic("computations_materialized",this->getComputations(sourceLeaf,treeIdx));
#endif
                        pqueue.insert(childIdx + this->numNodes, dist);
                    }
                }
            }
        }
    }
#if defined(COLLECT_STATISTICS)
    this->stats.incrementStatistic("computations_total",this->stats.getStatistic("computations_materialized")+this->stats.getStatistic("computations_executed"));
#endif
}

bool AdaptiveGtree::getSourceLeafkNNsByINE(NodeID queryNode, unsigned int k, std::unordered_set<NodeID> &targets,
                                           int leafNode, Graph &graph, std::vector<NodeID> &kNNs,
                                           std::vector<EdgeWeight> &kNNDistances,
                                           BinaryMinHeap<EdgeWeight, NodeID> &pqueue)
{
    // Note: We assumpe vectors passed to this method are empty

    BinaryMinHeap<EdgeWeight, NodeID> localQueue;
    EdgeWeight minDist;
    NodeID minDistNodeID, adjNode;
//     std::vector<bool> isNodeSettled(graph.getNumNodes(),false);
//     // We use unordered_set because we are only searching subgraph (no benefit 
//     // from allocating std::vector<bool> for all nodes, just overhead)
    std::unordered_set<NodeID> settledNodeIDs;
    // We can use the fact that we store the leafVerticesVec of each NodeID
    // in the this->nodeIDLeafBordersVecIdx vector to use a vector<bool>
    // to store the visited status of leaf vertices during INE search
//     std::vector<bool> leafVertexVisited(this->treeNodes[leafNode].leafVerticesVec.size(),false);
    int adjListStart, nextAdjListStart;

    // Initialize with priority queue with query node ID
    localQueue.insert(queryNode, 0);

    bool borderEncountered = false;
    unsigned int targetsFound = 0;

    // We also exist if all target nodes have been found
    while (localQueue.size() > 0 && targetsFound < targets.size()) {
        // Extract and remove node with smallest distance from query point
        // and mark it as "settled" so we do not inspect again
        minDist = localQueue.getMinKey();
        minDistNodeID = localQueue.extractMinElement();
//         if (!isNodeSettled[minDistNodeID]) {
//             isNodeSettled[minDistNodeID] = 1;

        // if not visited
        if (settledNodeIDs.find(minDistNodeID) == settledNodeIDs.end()) {
            settledNodeIDs.insert(minDistNodeID);
//         if (!leafVertexVisited[this->nodeIDLeafVerticesVecIdx[minDistNodeID]]) {
//             leafVertexVisited[this->nodeIDLeafVerticesVecIdx[minDistNodeID]] = true;

            // if the node contains an object
            if (targets.find(minDistNodeID) != targets.end()) {
                // TODO: what if it contains more then one object?
                ++targetsFound;
                // If the minimum is an object we have found a kNN
                if (!borderEncountered) {
                    // If we have not encounted a border so far then
                    // all objects we encounter in the source leaf
                    // cannot be bettered by a object outside the source leaf
                    // so we can count these as kNN results
                    kNNs.push_back(minDistNodeID);
                    kNNDistances.push_back(minDist);
                } else {
                    pqueue.insert(minDistNodeID, minDist);
                }
                if (kNNs.size() == k) {
                    return true;
                } else if (targetsFound == k) {
                    // We do not need to add more than k candidates within leaf
                    // because at best these are the actual kNN neighbours and at
                    // worst some of them will be replaced by closer kNNs outside this leaf
                    break;
                }
            }

            // Inspect each neighbour and update pqueue using edge weights
            adjListStart = graph.getEdgeListStartIndex(minDistNodeID);
            nextAdjListStart = graph.getEdgeListSize(minDistNodeID);
            // Note: We have to make sure we don't exceed size of graph.edges vector

            bool isBorder = false;
            for (int i = adjListStart; i < nextAdjListStart; ++i) {
                adjNode = graph.edges[i].first;
                if (this->isEdgeInLeafSubgraph(i)) {
//                     if (!isNodeSettled[adjNode]) {
                    if (settledNodeIDs.find(adjNode) == settledNodeIDs.end()) {
//                     if (!leafVertexVisited[this->nodeIDLeafVerticesVecIdx[adjNode]]) {
                        // This is the first time we have seen this node (i.e. distance infinity)
                        localQueue.insert(adjNode, minDist + graph.edges[i].second);
                    }
                } else {
                    if (!isBorder) {
                        isBorder = true;
                    }
                    if (!borderEncountered) {
                        borderEncountered = true;
                    }
                }
            }

            // If it is a border we add each of the other borders for this leaf
            // node and the distance from this border (assuming they have not 
            // already been settled - if they are settled we have already found
            // shortest path to them so the path through this border cannot
            // be an improvement)
            if (isBorder) {
//                 int borderIdx = this->treeNodes[leafNode].getBorderIdxInBorderVec(minDistNodeID);
                int borderIdx = this->getIdxInLeafBordersVec(minDistNodeID);
                for (std::size_t i = 0; i < this->treeNodes[leafNode].bordersVec.size(); ++i) {
                    // Note since this is leaf the above functions actual gets idx in leafVerticesVec
                    // Recall that the dimension of the distance matrix is bordersVec.size()*leafVerticesVec.size()
                    // and stores the distances from borders to leaf vertices so we can use these values
                    // compute the distances between two borders in the leaf vertice
//                     if (!isNodeSettled[this->treeNodes[leafNode].bordersVec[i]]) {
                    if (settledNodeIDs.find(this->treeNodes[leafNode].bordersVec[i]) == settledNodeIDs.end()) {
//                     if (!leafVertexVisited[this->nodeIDLeafVerticesVecIdx[this->treeNodes[leafNode].bordersVec[i]]]) {
                        int targetBorderIdx = this->treeNodes[leafNode].getBorderIdxInChildBorderVec(i);
#if defined(GTREE_STL_HASH_TABLE_DIST_MATRIX) || defined(GTREE_GOOGLE_DENSEHASH_DIST_MATRIX)
                        localQueue.insert(this->treeNodes[leafNode].bordersVec[i],minDist+this->distanceMatrix[minDistNodeID][this->treeNodes[leafNode].bordersVec[i]]);
#else
                        if (!this->treeNodes[leafNode].distanceMatrix.isAssigned(borderIdx, targetBorderIdx)) {
                            DistanceMatrixBuilder::fillRow2(graph, this->treeNodes[leafNode], minDistNodeID, borderIdx,
                                                           this->treeNodes[leafNode].bordersVec);
                        }
                        localQueue.insert(this->treeNodes[leafNode].bordersVec[i], minDist +
                                                                                   this->treeNodes[leafNode].distanceMatrix.get(
                                                                                           borderIdx, targetBorderIdx));
#endif
                    }
                }
#if defined(COLLECT_STATISTICS)
                this->stats.incrementStatistic("computations_executed",this->treeNodes[leafNode].bordersVec.size());
#endif
            }
        }
    }
    return false;
}

EdgeWeight AdaptiveGtree::SPDistToSourceLeafNode(Graph &graph, NodeID u, int sourceLeafIdx)
{
    EdgeWeight spDist = 0, sourceToNextBorderDist;
#if defined(GTREE_STL_HASH_TABLE_DIST_MATRIX) || defined(GTREE_GOOGLE_DENSEHASH_DIST_MATRIX)
    if (this->treeNodes[sourceLeafIdx].bordersVec.size() > 0) {
        spDist = this->distanceMatrix[this->treeNodes[sourceLeafIdx].bordersVec[0]][u];
        this->sourceToTreeNodeBorderDist[sourceLeafIdx][0] = spDist;
        for (std::size_t i = 1; i < this->treeNodes[sourceLeafIdx].bordersVec.size(); ++i) {
            sourceToNextBorderDist = this->distanceMatrix[this->treeNodes[sourceLeafIdx].bordersVec[i]][u];
            // ASSUMING DIRECTED GRAPH (I.E. BORDER TO LEAF VERTEX DISTANCE = LEAF VERTEX TO BORDER DISTANCE
            // THIS IS BECAUSE WE HAVEN'T CALCULATED LEAF VERTEX TO BORDER DISTANCES
            this->sourceToTreeNodeBorderDist[sourceLeafIdx][i] = sourceToNextBorderDist;
            if (sourceToNextBorderDist < spDist) {
                spDist = sourceToNextBorderDist;
            }
        }
    }
#else
    int uIdx = this->getIdxInLeafVerticesVec(u);
    if (!this->treeNodes[sourceLeafIdx].bordersVec.empty()) {
        if (!this->treeNodes[sourceLeafIdx].distanceMatrix.isAssigned(0, uIdx)) {
            DistanceMatrixBuilder::fillColumn(graph, this->treeNodes[sourceLeafIdx].distanceMatrix, u,
                                              uIdx, this->treeNodes[sourceLeafIdx].bordersVec);
        }
        spDist = this->treeNodes[sourceLeafIdx].distanceMatrix.get(0, uIdx);
        this->sourceToTreeNodeBorderDist[sourceLeafIdx][0] = spDist;
        for (std::size_t i = 0; i < this->treeNodes[sourceLeafIdx].bordersVec.size(); ++i) {
            if (!this->treeNodes[sourceLeafIdx].distanceMatrix.isAssigned(i, uIdx)) {
                DistanceMatrixBuilder::fillColumn(graph, this->treeNodes[sourceLeafIdx].distanceMatrix, u,
                                                  uIdx, this->treeNodes[sourceLeafIdx].bordersVec);
            }
            sourceToNextBorderDist = this->treeNodes[sourceLeafIdx].distanceMatrix.get(i, uIdx);
            // ASSUMING DIRECTED GRAPH (I.E. BORDER TO LEAF VERTEX DISTANCE = LEAF VERTEX TO BORDER DISTANCE
            // THIS IS BECAUSE WE HAVEN'T CALCULATED LEAF VERTEX TO BORDER DISTANCES
            this->sourceToTreeNodeBorderDist[sourceLeafIdx][i] = sourceToNextBorderDist;
            if (sourceToNextBorderDist < spDist) {
                spDist = sourceToNextBorderDist;
            }
        }
    }
#endif
#if defined(COLLECT_STATISTICS)
    this->stats.incrementStatistic("computations_executed",this->treeNodes[sourceLeafIdx].bordersVec.size());
#endif
    return spDist;
}

EdgeWeight AdaptiveGtree::SPDistToParentNode(Graph & graph, int childTreeIdx, int parentTreeIdx, bool computeSPDist)
{
    Logger::log("SPDistToParentNode start");
    EdgeWeight spDist = 0, sourceToNextBorderDist, sourceToChildBorderDist, sourceToParentBorderDist;

    // Note: We assume we have already calculated distance to childTreeIdx from source

#if defined(GTREE_STL_HASH_TABLE_DIST_MATRIX) || defined(GTREE_GOOGLE_DENSEHASH_DIST_MATRIX)
    if (this->treeNodes[childTreeIdx].bordersVec.size() > 0) {
        sourceToChildBorderDist = this->sourceToTreeNodeBorderDist[childTreeIdx][0];
        for (std::size_t k = 0; k < this->treeNodes[parentTreeIdx].bordersVec.size(); ++k) {
            sourceToParentBorderDist = sourceToChildBorderDist + 
                this->distanceMatrix[this->treeNodes[childTreeIdx].bordersVec[0]][this->treeNodes[parentTreeIdx].bordersVec[k]];
            // I.e. this is the first time we are computing distances to parent's border set so we initialise
            this->sourceToTreeNodeBorderDist[parentTreeIdx][k] = sourceToParentBorderDist;
        }
        for (std::size_t j = 1; j < this->treeNodes[childTreeIdx].bordersVec.size(); ++j) {
            sourceToChildBorderDist = this->sourceToTreeNodeBorderDist[childTreeIdx][j];
            for (std::size_t k = 0; k < this->treeNodes[parentTreeIdx].bordersVec.size(); ++k) {
                sourceToParentBorderDist = sourceToChildBorderDist + 
                    this->distanceMatrix[this->treeNodes[childTreeIdx].bordersVec[j]][this->treeNodes[parentTreeIdx].bordersVec[k]];
                if (sourceToParentBorderDist < this->sourceToTreeNodeBorderDist[parentTreeIdx][k]) {
                    this->sourceToTreeNodeBorderDist[parentTreeIdx][k] = sourceToParentBorderDist;
                }
            }
        }
    }
#else
    int childPos, parentBorderIdx, childBorderOffset, childBorderIdx;

    childPos = this->treeNodes[childTreeIdx].getParentChildIdx();
    childBorderOffset = this->treeNodes[parentTreeIdx].getChildOffsetInChildBorderVec(childPos);

    if (this->treeNodes[childTreeIdx].bordersVec.size() > 0) {
        childBorderIdx = childBorderOffset;
        sourceToChildBorderDist = this->sourceToTreeNodeBorderDist[childTreeIdx][0];
        for (std::size_t k = 0; k < this->treeNodes[parentTreeIdx].bordersVec.size(); ++k) {
            parentBorderIdx = this->treeNodes[parentTreeIdx].getBorderIdxInChildBorderVec(k);
            if (!this->treeNodes[parentTreeIdx].distanceMatrix.isAssigned(childBorderIdx, parentBorderIdx)) {
                Logger::log(" > Distance matrix not assigned");
                DistanceMatrixBuilder::fillRow2(graph, this->treeNodes[parentTreeIdx],
                                               this->treeNodes[childTreeIdx].bordersVec[0],
                                               childBorderIdx,
                                               this->treeNodes[parentTreeIdx].bordersVec);
            }
            sourceToParentBorderDist = sourceToChildBorderDist +
                                       this->treeNodes[parentTreeIdx].distanceMatrix.get(childBorderIdx,
                                                                                         parentBorderIdx);
            // I.e. this is the first time we are computing distances to parent's border set so we initialise
            this->sourceToTreeNodeBorderDist[parentTreeIdx][k] = sourceToParentBorderDist;
        }
        for (std::size_t j = 0; j < this->treeNodes[childTreeIdx].bordersVec.size(); ++j) {
            Logger::log(" > For ", j);
            childBorderIdx = childBorderOffset + j;
            sourceToChildBorderDist = this->sourceToTreeNodeBorderDist[childTreeIdx][j];
            for (std::size_t k = 0; k < this->treeNodes[parentTreeIdx].bordersVec.size(); ++k) {
                Logger::log(" >   Inner for ", k);
                parentBorderIdx = this->treeNodes[parentTreeIdx].getBorderIdxInChildBorderVec(k);
                if (!this->treeNodes[parentTreeIdx].distanceMatrix.isAssigned(childBorderIdx, parentBorderIdx)) {
                    Logger::log(" >   Distance matrix not assigned");
                    DistanceMatrixBuilder::fillRow2(graph, this->treeNodes[parentTreeIdx],
                                                   this->treeNodes[childTreeIdx].bordersVec[j],
                                                   childBorderIdx,
                                                   this->treeNodes[parentTreeIdx].bordersVec);
                }
                sourceToParentBorderDist = sourceToChildBorderDist +
                                           this->treeNodes[parentTreeIdx].distanceMatrix.get(childBorderIdx,
                                                                                             parentBorderIdx);
                if (sourceToParentBorderDist < this->sourceToTreeNodeBorderDist[parentTreeIdx][k]) {
                    this->sourceToTreeNodeBorderDist[parentTreeIdx][k] = sourceToParentBorderDist;
                }
            }
        }
    }
#endif

    if (this->sourceToTreeNodeBorderDist[parentTreeIdx].size() > 0 && computeSPDist) {
        spDist = this->sourceToTreeNodeBorderDist[parentTreeIdx][0];
        for (std::size_t i = 1; i < this->sourceToTreeNodeBorderDist[parentTreeIdx].size(); ++i) {
            sourceToNextBorderDist = this->sourceToTreeNodeBorderDist[parentTreeIdx][i];
            if (sourceToNextBorderDist < spDist) {
                spDist = sourceToNextBorderDist;
            }
        }
    }

#if defined(COLLECT_STATISTICS)
    this->stats.incrementStatistic("computations_executed",this->treeNodes[childTreeIdx].bordersVec.size()*this->treeNodes[parentTreeIdx].bordersVec.size());
#endif
    return spDist;
}

EdgeWeight
AdaptiveGtree::SPDistToSiblingNode(Graph &graph, int firstLCAChildIdx, int targetLCAChildIdx, int LCAIdx, bool computeSPDist)
{
    Logger::log("SPDistToSiblingNode start");
    EdgeWeight spDist = 0, sourceToNextBorderDist, sourceToChildBorderDist, sourceToBorderDist;

#if defined(GTREE_STL_HASH_TABLE_DIST_MATRIX) || defined(GTREE_GOOGLE_DENSEHASH_DIST_MATRIX)
    if (this->treeNodes[firstLCAChildIdx].bordersVec.size() > 0) {
        sourceToChildBorderDist = this->sourceToTreeNodeBorderDist[firstLCAChildIdx][0];
        for (std::size_t k = 0; k < this->treeNodes[targetLCAChildIdx].bordersVec.size(); ++k) {
            sourceToBorderDist = sourceToChildBorderDist + 
                this->distanceMatrix[this->treeNodes[firstLCAChildIdx].bordersVec[0]][this->treeNodes[targetLCAChildIdx].bordersVec[k]];
            // I.e. this is the first time we are computing distances to tthe target set
            this->sourceToTreeNodeBorderDist[targetLCAChildIdx][k] = sourceToBorderDist;
        }
        for (std::size_t j = 1; j < this->treeNodes[firstLCAChildIdx].bordersVec.size(); ++j) {
            sourceToChildBorderDist = this->sourceToTreeNodeBorderDist[firstLCAChildIdx][j];
            for (std::size_t k = 0; k < this->treeNodes[targetLCAChildIdx].bordersVec.size(); ++k) {
                sourceToBorderDist = sourceToChildBorderDist + 
                    this->distanceMatrix[this->treeNodes[firstLCAChildIdx].bordersVec[j]][this->treeNodes[targetLCAChildIdx].bordersVec[k]];
                if (sourceToBorderDist < this->sourceToTreeNodeBorderDist[targetLCAChildIdx][k]) {
                    this->sourceToTreeNodeBorderDist[targetLCAChildIdx][k] = sourceToBorderDist;
                }
            }
        }
    }
#else
    int childPos, childBorderOffset, childBorderIdx;
    int targetChildPos, targetBorderOffset, targetBorderIdx;

    childPos = this->treeNodes[firstLCAChildIdx].getParentChildIdx();
    childBorderOffset = this->treeNodes[LCAIdx].getChildOffsetInChildBorderVec(childPos);
    targetChildPos = this->treeNodes[targetLCAChildIdx].getParentChildIdx();
    targetBorderOffset = this->treeNodes[LCAIdx].getChildOffsetInChildBorderVec(targetChildPos);

    if (this->treeNodes[firstLCAChildIdx].bordersVec.size() > 0) {
        childBorderIdx = childBorderOffset;
        sourceToChildBorderDist = this->sourceToTreeNodeBorderDist[firstLCAChildIdx][0];
        for (std::size_t k = 0; k < this->treeNodes[targetLCAChildIdx].bordersVec.size(); ++k) {
            targetBorderIdx = targetBorderOffset + k;
            if (!this->treeNodes[LCAIdx].distanceMatrix.isAssigned(childBorderIdx, targetBorderIdx)) {
                Logger::log("SPDistToSiblingNode distanceMatrix not assigned");
                DistanceMatrixBuilder::fillRow3(graph, this->treeNodes[LCAIdx],
                                               this->treeNodes[firstLCAChildIdx].bordersVec[0],
                                               childBorderIdx,
                                               this->treeNodes[targetLCAChildIdx].bordersVec,
                                                targetBorderOffset);
            }
            sourceToBorderDist = sourceToChildBorderDist +
                                 this->treeNodes[LCAIdx].distanceMatrix.get(childBorderIdx, targetBorderIdx);
            // I.e. this is the first time we are computing distances to tthe target set
            this->sourceToTreeNodeBorderDist[targetLCAChildIdx][k] = sourceToBorderDist;
        }
        for (std::size_t j = 1; j < this->treeNodes[firstLCAChildIdx].bordersVec.size(); ++j) {
            childBorderIdx = childBorderOffset + j;
            sourceToChildBorderDist = this->sourceToTreeNodeBorderDist[firstLCAChildIdx][j];
            for (std::size_t k = 0; k < this->treeNodes[targetLCAChildIdx].bordersVec.size(); ++k) {
                targetBorderIdx = targetBorderOffset + k;
                if (!this->treeNodes[LCAIdx].distanceMatrix.isAssigned(childBorderIdx, targetBorderIdx)) {
                    DistanceMatrixBuilder::fillRow3(graph, this->treeNodes[LCAIdx],
                                                   this->treeNodes[firstLCAChildIdx].bordersVec[j],
                                                   childBorderIdx,
                                                   this->treeNodes[targetLCAChildIdx].bordersVec,
                                                   targetBorderOffset);
                }
                sourceToBorderDist = sourceToChildBorderDist +
                                     this->treeNodes[LCAIdx].distanceMatrix.get(childBorderIdx, targetBorderIdx);
                if (sourceToBorderDist < this->sourceToTreeNodeBorderDist[targetLCAChildIdx][k]) {
                    this->sourceToTreeNodeBorderDist[targetLCAChildIdx][k] = sourceToBorderDist;
                }
            }
        }
    }
#endif

    if (this->sourceToTreeNodeBorderDist[targetLCAChildIdx].size() > 0 && computeSPDist) {
        spDist = this->sourceToTreeNodeBorderDist[targetLCAChildIdx][0];
        for (std::size_t i = 1; i < this->sourceToTreeNodeBorderDist[targetLCAChildIdx].size(); ++i) {
            sourceToNextBorderDist = this->sourceToTreeNodeBorderDist[targetLCAChildIdx][i];
            if (sourceToNextBorderDist < spDist) {
                spDist = sourceToNextBorderDist;
            }
        }
    }
#if defined(COLLECT_STATISTICS)
    this->stats.incrementStatistic("computations_executed",this->treeNodes[firstLCAChildIdx].bordersVec.size()*this->treeNodes[targetLCAChildIdx].bordersVec.size());
#endif

    return spDist;
}

EdgeWeight AdaptiveGtree::SPDistToChildNode(Graph &graph, int childTreeIdx, int parentTreeIdx, bool computeSPDist)
{

    EdgeWeight spDist = 0, sourceToNextBorderDist, sourceToChildBorderDist, sourceToBorderDist;

#if defined(GTREE_STL_HASH_TABLE_DIST_MATRIX) || defined(GTREE_GOOGLE_DENSEHASH_DIST_MATRIX)
    if (this->treeNodes[parentTreeIdx].bordersVec.size() > 0) {
        sourceToChildBorderDist = this->sourceToTreeNodeBorderDist[parentTreeIdx][0];
        for (std::size_t k = 0; k < this->treeNodes[childTreeIdx].bordersVec.size(); ++k) {
            sourceToBorderDist = sourceToChildBorderDist + 
                this->distanceMatrix[this->treeNodes[parentTreeIdx].bordersVec[0]][this->treeNodes[childTreeIdx].bordersVec[k]];
            // I.e. this is the first time we are computing distances to tthe target set
            this->sourceToTreeNodeBorderDist[childTreeIdx][k] = sourceToBorderDist;
        }
        for (std::size_t j = 1; j < this->treeNodes[parentTreeIdx].bordersVec.size(); ++j) {
            sourceToChildBorderDist = this->sourceToTreeNodeBorderDist[parentTreeIdx][j];
            for (std::size_t k = 0; k < this->treeNodes[childTreeIdx].bordersVec.size(); ++k) {
                sourceToBorderDist = sourceToChildBorderDist + 
                    this->distanceMatrix[this->treeNodes[parentTreeIdx].bordersVec[j]][this->treeNodes[childTreeIdx].bordersVec[k]];
                if (sourceToBorderDist < this->sourceToTreeNodeBorderDist[childTreeIdx][k]) {
                    this->sourceToTreeNodeBorderDist[childTreeIdx][k] = sourceToBorderDist;
                }
            }
        }
    }
#else
    int childPos, parentBorderIdx, childBorderOffset, childBorderIdx;
    childPos = this->treeNodes[childTreeIdx].getParentChildIdx();
    childBorderOffset = this->treeNodes[parentTreeIdx].getChildOffsetInChildBorderVec(childPos);

    if (this->treeNodes[parentTreeIdx].bordersVec.size() > 0) {
        parentBorderIdx = this->treeNodes[parentTreeIdx].getBorderIdxInChildBorderVec(0);
        sourceToChildBorderDist = this->sourceToTreeNodeBorderDist[parentTreeIdx][0];
        for (std::size_t k = 0; k < this->treeNodes[childTreeIdx].bordersVec.size(); ++k) {
            childBorderIdx = childBorderOffset + k;
            if (!this->treeNodes[parentTreeIdx].distanceMatrix.isAssigned(parentBorderIdx, childBorderIdx)) {
                DistanceMatrixBuilder::fillRow3(graph, this->treeNodes[parentTreeIdx],
                                               this->treeNodes[parentTreeIdx].bordersVec[0],
                                               parentBorderIdx,
                                               this->treeNodes[childTreeIdx].bordersVec,
                                                childBorderOffset);
            }
            sourceToBorderDist = sourceToChildBorderDist +
                                 this->treeNodes[parentTreeIdx].distanceMatrix.get(parentBorderIdx, childBorderIdx);
            // I.e. this is the first time we are computing distances to tthe target set
            this->sourceToTreeNodeBorderDist[childTreeIdx][k] = sourceToBorderDist;
        }
        for (std::size_t j = 1; j < this->treeNodes[parentTreeIdx].bordersVec.size(); ++j) {
            parentBorderIdx = this->treeNodes[parentTreeIdx].getBorderIdxInChildBorderVec(j);
            sourceToChildBorderDist = this->sourceToTreeNodeBorderDist[parentTreeIdx][j];
            for (std::size_t k = 0; k < this->treeNodes[childTreeIdx].bordersVec.size(); ++k) {
                childBorderIdx = childBorderOffset + k;
                if (!this->treeNodes[parentTreeIdx].distanceMatrix.isAssigned(parentBorderIdx, childBorderIdx)) {
                    DistanceMatrixBuilder::fillRow3(graph, this->treeNodes[parentTreeIdx],
                                                   this->treeNodes[parentTreeIdx].bordersVec[j],
                                                   parentBorderIdx,
                                                   this->treeNodes[childTreeIdx].bordersVec,
                                                   childBorderOffset);
                }
                sourceToBorderDist = sourceToChildBorderDist +
                                     this->treeNodes[parentTreeIdx].distanceMatrix.get(parentBorderIdx, childBorderIdx);
                if (sourceToBorderDist < this->sourceToTreeNodeBorderDist[childTreeIdx][k]) {
                    this->sourceToTreeNodeBorderDist[childTreeIdx][k] = sourceToBorderDist;
                }
            }
        }
    }
#endif

    if (this->sourceToTreeNodeBorderDist[childTreeIdx].size() > 0 && computeSPDist) {
        spDist = this->sourceToTreeNodeBorderDist[childTreeIdx][0];
        for (std::size_t i = 1; i < this->sourceToTreeNodeBorderDist[childTreeIdx].size(); ++i) {
            sourceToNextBorderDist = this->sourceToTreeNodeBorderDist[childTreeIdx][i];
            if (sourceToNextBorderDist < spDist) {
                spDist = sourceToNextBorderDist;
            }
        }
    }
#if defined(COLLECT_STATISTICS)
    this->stats.incrementStatistic("computations_executed",this->treeNodes[parentTreeIdx].bordersVec.size()*this->treeNodes[childTreeIdx].bordersVec.size());
#endif

    return spDist;

}

EdgeWeight AdaptiveGtree::SPDistToLeafTarget(Graph &graph, NodeID target, int leafIdx)
{
    EdgeWeight spDist = 0, candidateSourceToTargetDist;

    // Now find the border which has the shortest distance to the target in the leaf node
#if defined(GTREE_STL_HASH_TABLE_DIST_MATRIX) || defined(GTREE_GOOGLE_DENSEHASH_DIST_MATRIX)
    if (this->treeNodes[leafIdx].bordersVec.size() > 0) {
        spDist = this->sourceToTreeNodeBorderDist[leafIdx][0] + this->distanceMatrix[this->treeNodes[leafIdx].bordersVec[0]][target];
        for (std::size_t i = 0; i < this->treeNodes[leafIdx].bordersVec.size(); ++i) {
            candidateSourceToTargetDist = this->sourceToTreeNodeBorderDist[leafIdx][i] + this->distanceMatrix[this->treeNodes[leafIdx].bordersVec[i]][target];
            if (candidateSourceToTargetDist < spDist) {
                spDist = candidateSourceToTargetDist;
            }
        }
    }
#else
    int vIdx = this->getIdxInLeafVerticesVec(target);

    if (this->treeNodes[leafIdx].bordersVec.size() > 0) {
        std::unordered_map<NodeID, EdgeWeight> siblingBorderDistances;
        std::vector<NodeID> targetsVec = this->treeNodes[leafIdx].bordersVec;

        if (!this->treeNodes[leafIdx].distanceMatrix.isAssigned(0, vIdx)) {
            DistanceMatrixBuilder::fillColumn(graph, this->treeNodes[leafIdx].distanceMatrix, target,
                                              vIdx, this->treeNodes[leafIdx].bordersVec);
        }
        spDist = this->sourceToTreeNodeBorderDist[leafIdx][0] + this->treeNodes[leafIdx].distanceMatrix.atIndex(vIdx);
        for (std::size_t i = 0; i < this->treeNodes[leafIdx].bordersVec.size(); ++i) {
            if (!this->treeNodes[leafIdx].distanceMatrix.isAssigned(i, vIdx)) {
                DistanceMatrixBuilder::fillColumn(graph, this->treeNodes[leafIdx].distanceMatrix, target,
                                                  vIdx, this->treeNodes[leafIdx].bordersVec);
            }

            candidateSourceToTargetDist =
                    this->sourceToTreeNodeBorderDist[leafIdx][i] + this->treeNodes[leafIdx].distanceMatrix.get(i, vIdx);
            if (candidateSourceToTargetDist < spDist) {
                spDist = candidateSourceToTargetDist;
            }
        }
    }
#endif
#if defined(COLLECT_STATISTICS)
    this->stats.incrementStatistic("computations_executed",this->treeNodes[leafIdx].bordersVec.size());
#endif

    return spDist;
}

int AdaptiveGtree::getIdxInLeafVerticesVec(NodeID u)
{
    //assert(this->nodeIDLeafVerticesVecIdx[u] != -1 && "This node was never added to a leaf Gtree node");
    return this->nodeIDLeafVerticesVecIdx[u];
}

void AdaptiveGtree::setLeafVerticesVecIdx(NodeID u, int leafVerticesVecIdx)
{
    this->nodeIDLeafVerticesVecIdx[u] = leafVerticesVecIdx;
}

int AdaptiveGtree::getIdxInLeafBordersVec(NodeID u)
{
    return this->nodeIDLeafBordersVecIdx[u];
}

void AdaptiveGtree::setLeafBordersVecIdx(NodeID u, int leafBordersVecIdx)
{
    this->nodeIDLeafBordersVecIdx[u] = leafBordersVecIdx;
}

bool AdaptiveGtree::isEdgeInLeafSubgraph(NodeID edge)
{
    return this->edgeInLeafSubgraph[edge];
}

void AdaptiveGtree::setEdgeNotInSubgraph(NodeID edge)
{
    this->edgeInLeafSubgraph[edge] = false;
}

double AdaptiveGtree::computeIndexSize()
{
    double memoryUsage = 0;
    for (std::size_t i = 0; i < this->treeNodes.size(); ++i) {
        memoryUsage += this->treeNodes[i].computeIndexSizeBytes();
    }
    memoryUsage += sizeof(int) * this->leafIdxs.size();
    memoryUsage += sizeof(std::vector<EdgeWeight>) * this->sourceToTreeNodeBorderDist.size();
    for (std::size_t i = 0; i < sourceToTreeNodeBorderDist.size(); ++i) {
        memoryUsage += sizeof(EdgeWeight) * this->sourceToTreeNodeBorderDist[i].size();
    }
    memoryUsage += sizeof(int) * this->nodeIDLeafVerticesVecIdx.size();
    memoryUsage += sizeof(int) * this->nodeIDLeafBordersVecIdx.size();
    memoryUsage += this->edgeInLeafSubgraph.size() / 8; // std::vector<bool> only use 1 bit per element
    memoryUsage += sizeof(int) * this->nodeLeafIdxs.size();
    return memoryUsage / (1024 * 1024);
}

double AdaptiveGtree::computeMemoryUsage()
{
    double memoryUsage = 0;
    memoryUsage += sizeof(*this);
    for (std::size_t i = 0; i < this->treeNodes.size(); ++i) {
        memoryUsage += this->treeNodes[i].computeMemoryUsageBytes();
    }
    memoryUsage += sizeof(AdaptiveGtreeNode) * (this->treeNodes.capacity() - this->treeNodes.size());
    memoryUsage += sizeof(int) * this->leafIdxs.capacity();
    memoryUsage += sizeof(std::vector<EdgeWeight>) * this->sourceToTreeNodeBorderDist.capacity();
    for (std::size_t i = 0; i < sourceToTreeNodeBorderDist.size(); ++i) {
        memoryUsage += sizeof(EdgeWeight) * this->sourceToTreeNodeBorderDist[i].capacity();
    }
    memoryUsage += sizeof(int) * this->nodeIDLeafVerticesVecIdx.capacity();
    memoryUsage += sizeof(int) * this->nodeIDLeafBordersVecIdx.capacity();
    memoryUsage += this->edgeInLeafSubgraph.capacity() / 8; // std::vector<bool> only use 1 bit per element
    memoryUsage += sizeof(int) * this->nodeLeafIdxs.capacity();
    memoryUsage += sizeof(NodeID) * this->METISIdxToNodeID.capacity();
    memoryUsage += this->networkName.size();
#if defined(GTREE_STL_HASH_TABLE_DIST_MATRIX) || defined(GTREE_GOOGLE_DENSEHASH_DIST_MATRIX)
    memoryUsage += utility::estimateUnorderedMapMemoryUsageBytes(this->distanceMatrix.size(),sizeof(std::pair<NodeID,std::unordered_map<NodeID,EdgeWeight>>),this->distanceMatrix.bucket_count());
    for (auto it = this->distanceMatrix.begin(); it != this->distanceMatrix.end(); ++it) {
        memoryUsage += sizeof(it->second)+utility::estimateUnorderedMapMemoryUsageBytes(it->second.size(),sizeof(std::pair<NodeID,EdgeWeight>),it->second.bucket_count());
    }
#endif
    return memoryUsage / (1024 * 1024);
}

double AdaptiveGtree::computeDistanceMatrixMemoryUsage()
{
    double memoryUsage = 0;
#if defined(GTREE_STL_HASH_TABLE_DIST_MATRIX) || defined(GTREE_GOOGLE_DENSEHASH_DIST_MATRIX)
    memoryUsage += utility::estimateUnorderedMapMemoryUsageBytes(this->distanceMatrix.size(),sizeof(std::pair<NodeID,std::unordered_map<NodeID,EdgeWeight>>),this->distanceMatrix.bucket_count());
    for (auto it = this->distanceMatrix.begin(); it != this->distanceMatrix.end(); ++it) {
        memoryUsage += sizeof(it->second)+utility::estimateUnorderedMapMemoryUsageBytes(it->second.size(),sizeof(std::pair<NodeID,EdgeWeight>),it->second.bucket_count());
    }
#else
    for (std::size_t i = 0; i < this->treeNodes.size(); ++i) {
        memoryUsage += this->treeNodes[i].computeDistanceMatrixMemoryUsageBytes();
    }
#endif
    return memoryUsage / (1024 * 1024);
}

EdgeWeight AdaptiveGtree::getRepeatedShortestPathDistance(Graph &graph, NodeID u, NodeID v, std::vector<bool> &visited,
                                                          std::vector<EdgeWeight> &leafVertexDistances,
                                                          BinaryMinHeap<EdgeWeight, NodeID> &pqueue,
                                                          std::unordered_set<NodeID> &leafVertexVisited)
{
#if defined(COLLECT_STATISTICS)
    this->stats.clear();
    this->stats.initialiseStatistic("computations_materialized",0);
    this->stats.initialiseStatistic("computations_executed",0);
    this->stats.initialiseStatistic("computations_total",0);
#endif
    EdgeWeight spDist = 0;
    int uLeaf = this->getLeafIndex(u);
    int vLeaf = this->getLeafIndex(v);

    if (uLeaf == vLeaf) {
        // This mean they are both in the same node
//         spDist = this->SPDistLeaf(u,v,uLeaf,graph);
        if (leafVertexDistances.size() == 0) {
            // This means repeated leaf search has not be initialised yet
            leafVertexDistances.resize(this->treeNodes[uLeaf].leafVerticesVec.size(), 0);
//             leafVertexVisited.resize(this->treeNodes[uLeaf].leafVerticesVec.size(),false);
            pqueue.insert(u, 0); // Insert source node into queue for leaf search
        }
        spDist = this->getRepeatedSourceLeafShortestPathDistance(graph, uLeaf, v, leafVertexDistances, pqueue,
                                                                 leafVertexVisited);
    } else {
        int LCAIdx = 0;
        std::vector<int> &uPathFromRoot = this->treeNodes[uLeaf].getGtreePathFromRoot();
        std::vector<int> &vPathFromRoot = this->treeNodes[vLeaf].getGtreePathFromRoot();

        // Since the two nodes are in different leaf nodes we can guarantee that
        // there is at least one more node (the parent of both) in the G-tree path
        // which we call the lowest common ancestor (LCA)

        // Search down Gtree from root until we find first ancestor that is different
        // this means the previous ancestor is the LCA and the indexes point its children
        unsigned int i, j;
        for (i = 0, j = 0; i < uPathFromRoot.size() && j < vPathFromRoot.size(); ++i, ++j) {
            if (uPathFromRoot[i] != vPathFromRoot[j]) {
                // When i = 0 and j = 0 it is referring to the root so in that
                // case uPathFromRoot[i] does equal vPathFromRoot[j]. This means
                // when they are equal i > 0, so i-1 is safe here
                LCAIdx = uPathFromRoot[i - 1];
                break;
            }
            // Note: We can guarantee that LCAIdx will be set here. The only situation
            // it would not be set is both u and v were in the same leaf, but we have
            // guaranteed that is not the case in the if/else statement
        }

        // Now search up G-tree (from source leaf) until we reach i and j, then we 
        // search down (to target leaf) computing the shortest path distance

        // From source to source leaf
        if (!visited[uLeaf]) {
            this->SPDistToSourceLeafNode(graph, u, uLeaf);
            visited[uLeaf] = true;
        }

        // From source leaf to first child of LCA
        int x = i; // This is safe, depth of tree is O(log(n)) and n is at most 24 million in US dataset
        for (int k = uPathFromRoot.size() - 1; k > x; --k) {
            // Since k > x and x is at worst 0, k-1 is safe here
            if (!visited[uPathFromRoot[k - 1]]) {
                this->SPDistToParentNode(graph, uPathFromRoot[k], uPathFromRoot[k - 1], false);
                visited[uPathFromRoot[k - 1]] = true;
            }
        }

        // From first child of LCA to second child of LCA
        if (!visited[vPathFromRoot[j]]) {
            this->SPDistToSiblingNode(graph, uPathFromRoot[i], vPathFromRoot[j], LCAIdx, false);
            visited[vPathFromRoot[j]] = true;
        }

        // From second child of LCA to target leaf
        for (std::size_t k = j; k < vPathFromRoot.size() - 1; ++k) {
            // Note the size()-1 in the above condition
            if (!visited[vPathFromRoot[k + 1]]) {
                this->SPDistToChildNode(graph, vPathFromRoot[k + 1], vPathFromRoot[k], false);
                visited[vPathFromRoot[k + 1]] = true;
            }
        }

        // We assume target has not been visited
        spDist = this->SPDistToLeafTarget(graph, v, vLeaf);
    }

    return spDist;
}

EdgeWeight AdaptiveGtree::getRepeatedSourceLeafShortestPathDistance(Graph &graph, int leafNode, NodeID v,
                                                                    std::vector<EdgeWeight> &leafVertexDistances,
                                                                    BinaryMinHeap<EdgeWeight, NodeID> &pqueue,
                                                                    std::unordered_set<NodeID> &leafVertexVisited)
{
    // Assume vectors have been resized for number of vertices in source leaf
    // and query vertex has been inserted into pqueue
    //assert(this->getLeafIndex()[v] != leafNode && "Target is not in source leaf node!");

    if (leafVertexVisited.find(v) != leafVertexVisited.end()) {
        // If it's been already visited in the Dijkstra's search
        // we can return the distance immediately
        return leafVertexDistances[this->nodeIDLeafVerticesVecIdx[v]];
    }

    EdgeWeight minDist;
    NodeID minDistNodeID, adjNode;
    int adjListStart, nextAdjListStart;

    // We also exist if all target nodes have been found
    while (pqueue.size() > 0) {
        // Extract and remove node with smallest distance from query point
        // and mark it as "settled" so we do not inspect again
        minDist = pqueue.getMinKey();
        minDistNodeID = pqueue.extractMinElement();
        if (leafVertexVisited.find(minDistNodeID) == leafVertexVisited.end()) {
            if (minDistNodeID == v) {
                // Re-insert it back into queue so that we can dequeue
                // and add neighbours in repeated search
                pqueue.insert(minDistNodeID, minDist);
                return minDist;
            }

            leafVertexVisited.insert(minDistNodeID);
            leafVertexDistances[this->nodeIDLeafVerticesVecIdx[minDistNodeID]] = minDist;

            // Inspect each neighbour and update pqueue using edge weights
            adjListStart = graph.getEdgeListStartIndex(minDistNodeID);
            nextAdjListStart = graph.getEdgeListSize(minDistNodeID);
            // Note: We have to make sure we don't exceed size of graph.edges vector

            bool isBorder = false;
            for (int i = adjListStart; i < nextAdjListStart; ++i) {
                adjNode = graph.edges[i].first;
                if (this->isEdgeInLeafSubgraph(i)) {
                    if (leafVertexVisited.find(adjNode) == leafVertexVisited.end()) {
                        // This is the first time we have seen this node (i.e. distance infinity)
                        pqueue.insert(adjNode, minDist + graph.edges[i].second);
                    }
                } else {
                    if (!isBorder) {
                        isBorder = true;
                    }
                }
            }

            // If it is a border we add each of the other borders for this leaf
            // node and the distance from this border (assuming they have not 
            // already been settled - if they are settled we have already found
            // shortest path to them so the path through this border cannot
            // be an improvement)
            if (isBorder) {
                int borderIdx = this->getIdxInLeafBordersVec(minDistNodeID);
                for (std::size_t i = 0; i < this->treeNodes[leafNode].bordersVec.size(); ++i) {
                    // Note since this is leaf the above functions actual gets idx in leafVerticesVec
                    // Recall that the dimension of the distance matrix is bordersVec.size()*leafVerticesVec.size()
                    // and stores the distances from borders to leaf vertices so we can use these values
                    // compute the distances between two borders in the leaf vertice
                    if (leafVertexVisited.find(this->treeNodes[leafNode].bordersVec[i]) == leafVertexVisited.end()) {
                        int targetBorderIdx = this->treeNodes[leafNode].getBorderIdxInChildBorderVec(i);
#if defined(GTREE_STL_HASH_TABLE_DIST_MATRIX) || defined(GTREE_GOOGLE_DENSEHASH_DIST_MATRIX)
                        pqueue.insert(this->treeNodes[leafNode].bordersVec[i],minDist+this->distanceMatrix[minDistNodeID][this->treeNodes[leafNode].bordersVec[i]]);
#else
                        pqueue.insert(this->treeNodes[leafNode].bordersVec[i], minDist +
                                                                               this->treeNodes[leafNode].distanceMatrix.get(
                                                                                       borderIdx, targetBorderIdx));
#endif
                    }
                }
#if defined(COLLECT_STATISTICS)
                this->stats.incrementStatistic("computations_executed",this->treeNodes[leafNode].bordersVec.size());
#endif
            }

        }
    }
    assert(false && "Could not find target in G-tree source leaf node!");
    return 0;
}


void AdaptiveGtree::populateUnorderedMapDistanceMatrix()
{
#if defined(GTREE_GOOGLE_DENSEHASH_DIST_MATRIX)
    this->distanceMatrix.set_empty_key(constants::UNUSED_NODE_ID); // Required by google::dense_hash_map
#endif

    std::vector<std::vector<int>> treeNodeLevel = this->getTreeNodesByLevel();

    for (std::size_t i = 0; i < treeNodeLevel.size(); ++i) {
        for (std::size_t j = 0; j < treeNodeLevel[i].size(); ++j) {
            int treeNodeIdx = treeNodeLevel[i][j];
            if (!this->treeNodes[treeNodeIdx].isLeafNode()) {
                // For each pair of children we retrieve the border-to-border distances
                for (std::size_t k = 0; k < this->treeNodes[treeNodeIdx].children.size(); ++k) {
                    int sourceChild = this->treeNodes[treeNodeIdx].children[k];
                    int sourceChildOffset = this->treeNodes[treeNodeIdx].getChildOffsetInChildBorderVec(k);
                    // For each child's border find the distances to each border of every other child
                    for (std::size_t l = 0; l < this->treeNodes[sourceChild].bordersVec.size(); ++l) {
                        int sourceBorderIdx =
                                sourceChildOffset + l; // Position of source border in parent's distance matrix
#if defined(GTREE_GOOGLE_DENSEHASH_DIST_MATRIX)
                        if (this->distanceMatrix.find(this->treeNodes[sourceChild].bordersVec[l]) == this->distanceMatrix.end()) {
                            this->distanceMatrix[this->treeNodes[sourceChild].bordersVec[l]].set_empty_key(constants::UNUSED_NODE_ID);
                        }
#endif
                        for (std::size_t m = 0; m < this->treeNodes[treeNodeIdx].children.size(); ++m) {
                            int targetChild = this->treeNodes[treeNodeIdx].children[m];
                            int targetChildOffset = this->treeNodes[treeNodeIdx].getChildOffsetInChildBorderVec(m);
                            for (std::size_t n = 0; n < this->treeNodes[targetChild].bordersVec.size(); ++n) {
                                int targetBorderIdx = targetChildOffset + n;
                                this->distanceMatrix[this->treeNodes[sourceChild].bordersVec[l]][this->treeNodes[targetChild].bordersVec[n]] =
                                        this->treeNodes[treeNodeIdx].distanceMatrix.get(sourceBorderIdx,
                                                                                        targetBorderIdx);
                            }
                        }
                    }
                }
            } else {
                // For leaf nodes we retrieve the distance from each border to each leaf vertice
                for (std::size_t k = 0; k < this->treeNodes[treeNodeIdx].bordersVec.size(); ++k) {
#if defined(GTREE_GOOGLE_DENSEHASH_DIST_MATRIX)
                    if (this->distanceMatrix.find(this->treeNodes[treeNodeIdx].bordersVec[k]) == this->distanceMatrix.end()) {
                        this->distanceMatrix[this->treeNodes[treeNodeIdx].bordersVec[k]].set_empty_key(constants::UNUSED_NODE_ID);
                    }
#endif
                    for (std::size_t l = 0; l < this->treeNodes[treeNodeIdx].leafVerticesVec.size(); ++l) {
                        this->distanceMatrix[this->treeNodes[treeNodeIdx].bordersVec[k]][this->treeNodes[treeNodeIdx].leafVerticesVec[l]] =
                                this->treeNodes[treeNodeIdx].distanceMatrix.get(k, l);
                    }
                }
            }
        }
    }
}

int AdaptiveGtree::getComputations(int leafIdx, int targetIdx)
{
    int firstLCAChild;
    std::vector<int> gtreePath = this->getGtreePath(leafIdx, targetIdx, firstLCAChild);

    int numComputations = 0;
    std::size_t x = 0;

    if (gtreePath.size() > 0) {
        assert (gtreePath.size() >= 2 &&
                "Invalid path - must contain at least two nodes e.g. two leaves or leaf and parent");
        numComputations += this->treeNodes[gtreePath[0]].getNumBorders();

        // Add computation up the path until we reach firstLCAChild
        for (x = 1; x < gtreePath.size() && gtreePath[x - 1] != firstLCAChild; ++x) {
            numComputations += this->treeNodes[gtreePath[x - 1]].bordersVec.size() *
                               this->treeNodes[gtreePath[x]].bordersVec.size();
        }

        // Add computation between the two children of the LCA
        // Note: If the parent of leafIdx is the firstLCAChild then this loop does nothing
        // but x is now smaller than the gtreePath (because targetIdx is in the same path
        // as the leafIdx) then the following code does nothing
        if (x < gtreePath.size()) {
            numComputations += this->treeNodes[gtreePath[x - 1]].bordersVec.size() *
                               this->treeNodes[gtreePath[x]].bordersVec.size();
            ++x;
        }

        // Add computation down the path until we reach the end
        for (; x < gtreePath.size(); ++x) {
            numComputations += this->treeNodes[gtreePath[x - 1]].bordersVec.size() *
                               this->treeNodes[gtreePath[x]].bordersVec.size();
        }
    }

    return numComputations;
}