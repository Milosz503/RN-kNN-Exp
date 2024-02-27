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

#ifndef _ADAPTIVEGTREE_H
#define _ADAPTIVEGTREE_H

#include "../Graph.h"
#include "../Gtree.h"
#include "../../queue/BinaryMinHeap.h"
#include "../../utility/METISWrapper.h"
#include "../../utility/Statistics.h"
#include "AdaptiveGtreeNode.h"
#include "../DijkstraSearch.h"

#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/unordered_set.hpp>
#include <boost/serialization/unordered_map.hpp>
#if defined(GTREE_GOOGLE_DENSEHASH_DIST_MATRIX)
    #include <google/dense_hash_map>
#endif


class AdaptiveGtree {

    public:
        AdaptiveGtree(std::string networkName, int numNodes, int numEdges, int fanout, std::size_t maxLeafSize);
        AdaptiveGtree();
        std::string getNetworkName();
        int getNumNodes();
        int getNumEdges();
        int getFanout();
        int getTreeSize();
        int getLeafIndex(NodeID nodeID);
        int getParentIndex(int treeIdx);
        std::size_t getMaxLeafSize();
        void buildGtree(Graph& graph);
        void buildTreeHierarchy(Graph& graph);
        void computeDistanceMatrix(Graph& graph);
        void printLevels();
        int getNumLevels();
        int getNumBorders();
        int getBorderToBorderRelationships();
        int getAvgPathCost(int treeIdx = 0); // By default starts at root
        std::vector<std::vector<int>> getTreeNodesByLevel();
        void addNode(int parentTreeIdx, std::unordered_set<NodeID>& subgraph, Graph& originalGraph, METISWrapper& metis);
        void addChildren(int parentTreeIdx, std::unordered_set<NodeID>& parentSubgraph, Graph& originalGraph, METISWrapper& metis);
        EdgeWeight getShortestPathDistance(Graph& graph, NodeID u, NodeID v);
//        EdgeWeight SPDist(NodeID u, NodeID v, std::vector<int>& gtreePath, int firstLCAChild);
        EdgeWeight SPDistLeaf(NodeID u, NodeID v, int leafNode, Graph& graph);
        std::unordered_map<NodeID,EdgeWeight> DijkstraDistMultiTarget(NodeID u, std::unordered_set<NodeID>& targets, int leafNode, Graph& graph);
        EdgeWeight BorderDist(NodeID u, NodeID v, int leafNode);
        EdgeWeight DijkstraDist(NodeID u, NodeID v, int leafNode, Graph& graph);
        std::vector<int> getGtreePath(NodeID u, NodeID v, int& firstLCAChild);
        EdgeWeight SPDistToSourceLeafNode(Graph& graph, NodeID u, int treeIdx);
        EdgeWeight SPDistToParentNode(Graph &graph, int childTreeIdx, int parentTreeIdx, bool computeSPDist = true);
        EdgeWeight SPDistToSiblingNode(Graph &graph, int firstLCAChildIdx, int targetLCAChildIdx, int LCAIdx, bool computeSPDist = true);
        EdgeWeight SPDistToChildNode(Graph &graph, int childTreeIdx, int parentTreeIdx, bool computeSPDist = true);
        EdgeWeight SPDistToLeafTarget(Graph& graph, NodeID target, int leafIdx);
        void getKNNs(OccurenceList& occList, unsigned int k, NodeID queryNodeID, std::vector<NodeID>& kNNs,
                 std::vector<EdgeWeight>& kNNDistances, Graph& graph);
        bool getSourceLeafkNNsByINE(NodeID queryNode, unsigned int k, std::unordered_set<NodeID>& targets, int leafNode, Graph& graph, std::vector<NodeID>& kNNs,
                                    std::vector<EdgeWeight>& kNNDistances, BinaryMinHeap<EdgeWeight,NodeID>& pqueue);
        void initialiseGtreeQueryStructure();
        void printNode(int treeIdx);
        int getIdxInLeafVerticesVec(NodeID u);
        void setLeafVerticesVecIdx(NodeID u, int leafVerticesVecIdx);
        int getIdxInLeafBordersVec(NodeID u);
        void setLeafBordersVecIdx(NodeID u, int leafBordersVecIdx);
        bool isEdgeInLeafSubgraph(NodeID edge);
        void setEdgeNotInSubgraph(NodeID edge);
        double computeIndexSize();
        double computeMemoryUsage();
        double computeDistanceMatrixMemoryUsage();
        void printInfo();
        int getComputations(int leafIdx, int currIdx);

        // getRepeatedShortestPathDistance is used to find the shortest path distance to multiple
        // target from the same query node. This assumes the G-tree index is not being used
        // concurrently for difference query nodes (currently it does not need to be). The visited
        // vector indicates the G-tree nodes (by ID) that have been visited in previous calls
        // to this function. To start query for a new node, we need only assign all elements
        // in this vector to be false.
        EdgeWeight getRepeatedShortestPathDistance(Graph& graph, NodeID u, NodeID v, std::vector<bool>& visited, std::vector<EdgeWeight>& leafVertexDistances,
                                                   BinaryMinHeap<EdgeWeight,NodeID>& pqueue, std::unordered_set<NodeID>& leafVertexVisited);
        EdgeWeight getRepeatedSourceLeafShortestPathDistance(Graph& graph, int leafNode, NodeID v, std::vector<EdgeWeight>& leafVertexDistances,
                                                             BinaryMinHeap<EdgeWeight,NodeID>& pqueue, std::unordered_set<NodeID>& leafVertexVisited);

        // Build hash-table based distance matrices from existing vector-based distance matrices
        void populateUnorderedMapDistanceMatrix();

        std::vector<AdaptiveGtreeNode> treeNodes;
        std::vector<int> leafIdxs;
        // Note: The sourceToTreeNodeBorderDist is used to store the intermediary results during
        // querying using the G-tree index. Firstly this structure is included in the index size.
        // Secondly this structure can be re-used without needing to cleared for different
        // query nodes because G-tree queries traverse the hierarchy incrementally (source
        // to source leaf, child to parent, LCA sibling to LCA sibling, parent to child, and
        // finally target leaf to leaf). This means that when we want to calculate the
        // distances to a new G-tree node, we have already computed the distances to it's preceding node
        // (child, parent or sibling). That is it will always be overwritten for each new query node.
        // There may be incorrect distances in it (to a previous query node) but this data will
        // never be used until after it is overwritten for the current query node. Currently this is
        // true for getShortestPathDistance and getKNNs.
        std::vector<std::vector<EdgeWeight>> sourceToTreeNodeBorderDist;
        std::vector<int> nodeIDLeafVerticesVecIdx; // For each graph node we store the index occupied by that graph node in it's leaf Gtree node leafVerticesVec
        std::vector<int> nodeIDLeafBordersVecIdx; // For each graph node we store the index occupied by that graph node in it's leaf Gtree node bordersVec
        std::vector<bool> edgeInLeafSubgraph; // For each graph edge we store whether it is an edge that leads to another node in the subgraph to which the originating node belongs (use later to prune DijkstraDistMultiTarget)
        std::vector<int> nodeLeafIdxs; // For each graph node we store it's leaf Gtree node's index in the tree

        // Non-Serialized Members
        Statistics stats;

    private:
        friend class boost::serialization::access;

        DijkstraSearch dijkstra;
        std::string networkName;
        int numNodes;
        int numEdges;
        int fanout;
        std::size_t maxLeafSize;

        // Non-Serialized Members
        static const int ROOT_PARENT_INDEX = -1;
        std::vector<NodeID> METISIdxToNodeID;
#if defined(GTREE_GOOGLE_DENSEHASH_DIST_MATRIX)
        google::dense_hash_map<NodeID,google::dense_hash_map<NodeID,EdgeWeight>> distanceMatrix;
#else
        std::unordered_map<NodeID,std::unordered_map<NodeID,EdgeWeight>> distanceMatrix;
#endif

        // Boost Serialization
        template<class Archive>
        void serialize(Archive & ar, const unsigned int version)
        {
            ar & this->networkName;
            ar & this->numNodes;
            ar & this->numEdges;
            ar & this->fanout;
            ar & this->maxLeafSize;
            ar & this->treeNodes;
            ar & this->leafIdxs;
            ar & this->sourceToTreeNodeBorderDist;
            ar & this->nodeIDLeafVerticesVecIdx;
            ar & this->nodeIDLeafBordersVecIdx;
            ar & this->edgeInLeafSubgraph;
            ar & this->nodeLeafIdxs;
        }

};

#endif // _ADAPTIVEGTREE_H