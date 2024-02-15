//
// Created by milos on 14/02/2024.
//

#include "AdaptiveGtreeNode.h"

#include "../../utility/StopWatch.h"
#include "../../utility/utility.h"

#include <iostream>


/*
 * AdaptiveGtreeNode
 */

AdaptiveGtreeNode::AdaptiveGtreeNode(int treeIdx, int parentIdx, int numVertices):
        treeIdx(treeIdx), parentIdx(parentIdx), numVertices(numVertices), isLeaf(false) {}

AdaptiveGtreeNode::AdaptiveGtreeNode() {}

bool AdaptiveGtreeNode::isLeafNode()
{
    return this->isLeaf;
}

void AdaptiveGtreeNode::setLeafNode()
{
    this->isLeaf = true;
}

int AdaptiveGtreeNode::getParentIdx()
{
    return this->parentIdx;
}

int AdaptiveGtreeNode::getTreeIdx()
{
    return this->treeIdx;
}

int AdaptiveGtreeNode::getNumVertices()
{
    return this->numVertices;
}

int AdaptiveGtreeNode::getParentChildIdx()
{
    return this->parentChildIdx;
}

void AdaptiveGtreeNode::setParentChildIdx(int idx)
{
    this->parentChildIdx = idx;
}

int AdaptiveGtreeNode::addChild(int treeIdx)
{
    this->children.push_back(treeIdx);
    return this->children.size()-1;
}

std::vector<int>& AdaptiveGtreeNode::getChildren()
{
    return this->children;
}

int AdaptiveGtreeNode::addBorder(NodeID node)
{
    // Borders are only ever added to G-tree node once
    this->bordersVec.push_back(node);
    int bordersVecIdx = this->bordersVec.size()-1;
    if (!this->isLeaf) {
        // If this is not a leaf, it's childBordersVec will be full
        // so we want to remember the array index for this border in
        // childBordersVec
        this->borderOffsetsInChildBorderVec.push_back(childBorderToChildBorderVecIdx[node]);
    } else {
        this->borderOffsetsInChildBorderVec.push_back(leafVerticeToLeafVerticeVecIdx[node]);
    }
    return bordersVecIdx;
}

std::vector<NodeID>& AdaptiveGtreeNode::getBorders()
{
    return this->bordersVec;
}

int AdaptiveGtreeNode::getNumBorders()
{
    return this->bordersVec.size();
}

int AdaptiveGtreeNode::addLeafVertex(NodeID node)
{
    this->leafVerticesUset.insert(node);
    this->leafVerticesVec.push_back(node);
    int leafVerticesVecIdx = this->leafVerticesVec.size()-1;
    this->leafVerticeToLeafVerticeVecIdx[node] = leafVerticesVecIdx;
    return leafVerticesVecIdx;
}

bool AdaptiveGtreeNode::isLeafVertex(NodeID node)
{
    // If iterator is not equal to end, border exists
    return this->leafVerticesUset.find(node) != this->leafVerticesUset.end();
}

std::vector<NodeID>& AdaptiveGtreeNode::getLeafVertices()
{
    return this->leafVerticesVec;
}

std::unordered_set<NodeID >& AdaptiveGtreeNode::getLeafVerticesUset()
{
    return this->leafVerticesUset;
}

void AdaptiveGtreeNode::addChildBorder(NodeID node)
{
    // Child borders are only ever added once as they only belong to one child
    this->childBordersUset.insert(node);
    this->childBordersVec.push_back(node);
    this->childBorderToChildBorderVecIdx[node] = this->childBordersVec.size()-1;
}

bool AdaptiveGtreeNode::isChildBorder(NodeID node)
{
    // If iterator is not equal to end, border exists
    return this->childBordersUset.find(node) != this->childBordersUset.end();
}

std::vector<NodeID>& AdaptiveGtreeNode::getChildBorders()
{
    return this->childBordersVec;
}

std::unordered_set<NodeID>& AdaptiveGtreeNode::getChildBordersUset()
{
    return this->childBordersUset;
}

void AdaptiveGtreeNode::addGtreePathNode(int nodeIdx)
{
    this->gtreePath.push_back(nodeIdx);
}

void AdaptiveGtreeNode::addGtreePathNodes(const std::vector<int>& parentPath)
{
    this->gtreePath.insert(this->gtreePath.end(),parentPath.begin(),parentPath.end());
}

std::vector<int>& AdaptiveGtreeNode::getGtreePathFromRoot()
{
    return this->gtreePath;
}


int AdaptiveGtreeNode::getBorderIdxInChildBorderVec(int borderIdx)
{
    return this->borderOffsetsInChildBorderVec[borderIdx];
}

int AdaptiveGtreeNode::getChildOffsetInChildBorderVec(int childIdx)
{
    return this->childOffsetsInChildBorderVec[childIdx];
}

void AdaptiveGtreeNode::addChildOffsetInChildBorderVec()
{
    this->childOffsetsInChildBorderVec.push_back(this->childBordersVec.size());
}

void AdaptiveGtreeNode::printNode()
{
    std::cout << "Index: " << this->treeIdx << std::endl;
    std::cout << "Parent Index: " << this->parentIdx << std::endl;
    std::cout << "Parent's Child Vec Index: " << this->parentChildIdx << std::endl;
    std::cout << "isLeaf: " << this->isLeaf << std::endl  << std::endl;
    std::cout << "Children (" << this->children.size() << "): ";
    for (std::size_t i = 0; i < this->children.size(); ++i) {
        if (i != 0) {
            std::cout << ", ";
        }
        std::cout << i << " => " << children[i];
    }
    std::cout << std::endl << std::endl;
    std::cout << "Borders (" << this->bordersVec.size() << "): ";
    for (std::size_t i = 0; i < this->bordersVec.size(); ++i) {
        if (i != 0) {
            std::cout << ", ";
        }
        std::cout << i << " => " << bordersVec[i];
    }
    std::cout << std::endl << std::endl;
    std::cout << "Child Borders (" << this->childBordersVec.size() << "): ";
    for (std::size_t i = 0; i < this->childBordersVec.size(); ++i) {
        if (i != 0) {
            std::cout << ", ";
        }
        std::cout << i << " => " << childBordersVec[i];
    }
    std::cout << std::endl << std::endl;
    std::cout << "Border Offsets In Child Border Vec (" << this->borderOffsetsInChildBorderVec.size() << "): ";
    for (std::size_t i = 0; i < this->borderOffsetsInChildBorderVec.size(); ++i) {
        if (i != 0) {
            std::cout << ", ";
        }
        std::cout << i << " => " << borderOffsetsInChildBorderVec[i];
    }
    std::cout << std::endl << std::endl;
    std::cout << "Child Offsets In Child Border Vec (" << this->childOffsetsInChildBorderVec.size() << "): ";
    for (std::size_t i = 0; i < this->childOffsetsInChildBorderVec.size(); ++i) {
        if (i != 0) {
            std::cout << ", ";
        }
        std::cout << i << " => " << childOffsetsInChildBorderVec[i];
    }
    std::cout << std::endl << std::endl;
    std::cout << "Leaf Vertices (" << this->leafVerticesVec.size() << "): ";
    for (std::size_t i = 0; i < this->leafVerticesVec.size(); ++i) {
        if (i != 0) {
            std::cout << ", ";
        }
        std::cout << i << " => " << leafVerticesVec[i];
    }
    std::cout << std::endl << std::endl;
    std::cout << "Distance Matrix (" << this->distanceMatrix.size() << "): ";
    for (std::size_t i = 0; i < this->distanceMatrix.size(); ++i) {
        if (i != 0) {
            std::cout << ", ";
        }
        std::cout << i << " => " << distanceMatrix.atIndex(i);
    }
    std::cout << std::endl << std::endl;

}

double AdaptiveGtreeNode::computeIndexSizeBytes()
{
    double memoryUsage = 0;
    memoryUsage += sizeof(int)*5;
    memoryUsage += sizeof(bool);
    memoryUsage += sizeof(this->children);
    memoryUsage += sizeof(this->bordersVec);
    memoryUsage += sizeof(this->leafVerticesVec);
    memoryUsage += sizeof(this->gtreePath);
    memoryUsage += sizeof(this->distanceMatrix);
    memoryUsage += sizeof(this->borderOffsetsInChildBorderVec);
    memoryUsage += sizeof(this->childOffsetsInChildBorderVec);
    memoryUsage += sizeof(int)*this->children.size();
    memoryUsage += sizeof(NodeID)*this->bordersVec.size();
    memoryUsage += sizeof(NodeID)*this->leafVerticesVec.size();
    memoryUsage += sizeof(int)*this->gtreePath.size();
    memoryUsage += sizeof(EdgeWeight)*this->distanceMatrix.size();
    memoryUsage += sizeof(int)*this->borderOffsetsInChildBorderVec.size();
    memoryUsage += sizeof(int)*this->childOffsetsInChildBorderVec.size();
    return memoryUsage;
}

double AdaptiveGtreeNode::computeMemoryUsageBytes()
{
    double memoryUsage = 0;
    memoryUsage += sizeof(*this);
    memoryUsage += sizeof(int)*this->children.capacity();
    memoryUsage += sizeof(NodeID)*this->bordersVec.capacity();
    memoryUsage += sizeof(NodeID)*this->leafVerticesVec.capacity();
    memoryUsage += sizeof(NodeID)*this->childBordersVec.capacity();
    memoryUsage += utility::estimateUnorderedMapMemoryUsageBytes(this->childBordersUset.size(),sizeof(NodeID),this->childBordersUset.bucket_count());
    memoryUsage += utility::estimateUnorderedMapMemoryUsageBytes(this->leafVerticesUset.size(),sizeof(NodeID),this->leafVerticesUset.bucket_count());
    memoryUsage += sizeof(int)*this->gtreePath.capacity();
    memoryUsage += sizeof(EdgeWeight)*this->distanceMatrix.capacity();
    memoryUsage += sizeof(int)*this->borderOffsetsInChildBorderVec.capacity();
    memoryUsage += sizeof(int)*this->childOffsetsInChildBorderVec.capacity();
    memoryUsage += utility::estimateUnorderedMapMemoryUsageBytes(this->childBorderToChildBorderVecIdx.size(),sizeof(std::pair<NodeID,int>),this->childBorderToChildBorderVecIdx.bucket_count());
    memoryUsage += utility::estimateUnorderedMapMemoryUsageBytes(this->leafVerticeToLeafVerticeVecIdx.size(),sizeof(std::pair<NodeID,int>),this->leafVerticeToLeafVerticeVecIdx.bucket_count());
    return memoryUsage;
}

double AdaptiveGtreeNode::computeDistanceMatrixMemoryUsageBytes()
{
    double memoryUsage = 0;
    memoryUsage += sizeof(this->distanceMatrix);
    memoryUsage += sizeof(EdgeWeight)*this->distanceMatrix.capacity();
    return memoryUsage;
}