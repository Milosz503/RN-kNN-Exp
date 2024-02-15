//
// Created by milos on 14/02/2024.
//

#ifndef ND_KNN_ADAPTIVEGTREENODE_H
#define ND_KNN_ADAPTIVEGTREENODE_H

#include "../Graph.h"
#include "../Gtree.h"
#include "../../queue/BinaryMinHeap.h"
#include "../../utility/METISWrapper.h"
#include "../../utility/Statistics.h"
#include "DistanceMatrix.h"

#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/unordered_set.hpp>
#include <boost/serialization/unordered_map.hpp>


class AdaptiveGtreeNode {

public:
    AdaptiveGtreeNode(int treeIdx, int parentIdx, int numVertices);
    AdaptiveGtreeNode();
    bool isLeafNode();
    void setLeafNode();
    int getTreeIdx();
    int getParentIdx();
    int getNumVertices();
    int getParentChildIdx();
    void setParentChildIdx(int idx);
    int addChild(int treeIdx);
    std::vector<int>& getChildren();
    int addBorder(NodeID node);
    std::vector<NodeID>& getBorders();
    int getNumBorders();
    int addLeafVertex(NodeID node);
    bool isLeafVertex(NodeID node);
    std::vector<NodeID>& getLeafVertices();
    std::unordered_set<NodeID>& getLeafVerticesUset();
    void addChildBorder(NodeID node);
    bool isChildBorder(NodeID node);
    std::vector<NodeID>& getChildBorders();
    std::unordered_set<NodeID>& getChildBordersUset();
    void addGtreePathNode(int nodeIdx);
    void addGtreePathNodes(const std::vector<int>& parentPath);
    std::vector<int>& getGtreePathFromRoot();
    int getBorderIdxInChildBorderVec(int borderIdx);
    int getChildOffsetInChildBorderVec(int childIdx);
    void addChildOffsetInChildBorderVec();
    void printNode();
    double computeIndexSizeBytes();
    double computeMemoryUsageBytes();
    double computeDistanceMatrixMemoryUsageBytes();

    std::vector<int> children;
    std::vector<NodeID> bordersVec;
    std::vector<NodeID> leafVerticesVec;
    std::vector<int> gtreePath;
    DistanceMatrix distanceMatrix;
    // Corresponding childBordersVec index for each of this nodes borders (will have same size as bordersVec)
    // If it is the leaf node then it has the corresponding leafVerticesVec index
    // Note: These both cases correlate to the other order in the distanceMatrix vector
    std::vector<int> borderOffsetsInChildBorderVec;
    // The index of the first border of each of this node's children in childBordersVec (will have same size as fanout)
    std::vector<int> childOffsetsInChildBorderVec;
    int matrixRowLength;

    // Non-Serialized Members
    std::vector<NodeID> childBordersVec; // This doesn't need to serialized because we retrieve them from child nodes
    std::unordered_set<NodeID> leafVerticesUset;
    std::unordered_set<NodeID> childBordersUset;
    std::unordered_map<NodeID,int> childBorderToChildBorderVecIdx;
    std::unordered_map<NodeID,int> leafVerticeToLeafVerticeVecIdx;
    // Note: We only need these during construction (e.g. to build child offsets)

private:
    friend class boost::serialization::access;

    int treeIdx;
    int parentIdx;
    int numVertices;
    bool isLeaf;
    int parentChildIdx; // Index in parent child vector

    // Boost Serialization
    template<class Archive>
    void serialize(Archive & ar, const unsigned int version)
    {
        ar & this->treeIdx;
        ar & this->parentIdx;
        ar & this->numVertices;
        ar & this->isLeaf;
        ar & this->parentChildIdx;
        ar & this->children;
        ar & this->bordersVec;
        //ar & this->childBordersVec;
        ar & this->leafVerticesVec;
        //ar & this->bordersUset;
        //ar & this->childBordersUset;
        //ar & this->leafVerticesUset;
        ar & this->gtreePath;
        ar & this->distanceMatrix;
        ar & this->borderOffsetsInChildBorderVec;
        ar & this->childOffsetsInChildBorderVec;
        ar & this->matrixRowLength;
    }
};

#endif //ND_KNN_ADAPTIVEGTREENODE_H
