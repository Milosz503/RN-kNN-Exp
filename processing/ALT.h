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

#ifndef _ALT_H
#define _ALT_H

#include "Graph.h"
#include "Path.h"
#include "ier/ObjectList.h"
#include "../queue/BinaryMinHeap.h"

#include <boost/archive/binary_oarchive.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/iterator/counting_iterator.hpp>
#include <set>

enum LANDMARK_TYPE {
    RANDOM = 0,
    RANDOM_OBJECTS = 1,
    AVOID = 2,
    AVOID_PEQUE_URATA_IRYO = 3,
    ADVANCED_AVOID = 4,
    FARTHEST = 5,
    MIN_DIST = 6,
};

struct SizeNumNodesPair {
        unsigned long size = 0ul;
        unsigned numNodes = 0u;

        SizeNumNodesPair(unsigned long _size, unsigned _numNodes) : size(_size), numNodes(_numNodes) {}

        SizeNumNodesPair& operator += (const SizeNumNodesPair& obj) {
            this->size += obj.size;
            this->numNodes += obj.numNodes;
            return *this;
        }
};

struct ALTParameters {
    double threshold;
};

class ALT {

public:
    ALT(std::string networkName, int numNodes, int numEdges, ALTParameters parameters = {});

    ALT()
    {};

    void buildALT(Graph &graph, LANDMARK_TYPE landmarkType, unsigned int numLandmarks);

    void
    buildALT(Graph &graph, std::vector<NodeID> &objectNodes, LANDMARK_TYPE landmarkType, unsigned int numLandmarks);

    void generateMinDistLandmarks(Graph &graph, unsigned int maxNumLandmarks);

    double closestLandmarkNodesRatio(NodeID node,
                                      std::vector<std::vector<unsigned>>& landmarksPathLengths,
                                      std::vector<unsigned>& landmarksMaxPaths,
                                      unsigned numLandmarks
    );

    std::string getNetworkName();

    unsigned int getNumNodes();

    unsigned int getNumEdges();

    double computeIndexSize();

    EdgeWeight getLowerBound(NodeID s, NodeID t);
    EdgeWeight getLowerBound(NodeID s, NodeID t, std::vector<unsigned>& landmarkIndexes);

    EdgeWeight getLowestLowerBound(NodeID s, std::vector<NodeID>& targets);

    void getLowerAndUpperBound(NodeID s, NodeID t, EdgeWeight &lb, EdgeWeight &ub);

    Path findShortestPath(Graph &graph, NodeID source, NodeID target, std::vector<NodeID> &shortestPathTree);

    PathDistance findShortestPathDistance(Graph &graph, NodeID source, NodeID target);

    std::vector<std::pair<NodeID, EdgeWeight>>
    findShortestPathDistances(Graph &graph, NodeID source, std::vector<NodeID> &targets);

    LANDMARK_TYPE getLandmarkType(std::string selectionMethod, bool &success);

    NodeID getClosestCandidate(NodeID q);

    NodeID getNextCandidate(NodeID q, EdgeWeight &lbDistance);

    unsigned getLandmarksNumber() {
        return landmarks.size();
    }

    unsigned long edgesAccessedCount = 0;

    unsigned long calculateSizes(NodeID root, std::vector<std::tuple<std::vector<NodeID>, EdgeWeight, unsigned long>>& tree, std::vector<NodeID>& landmarks);
    NodeID getMaxLeaf(NodeID root, std::vector<std::tuple<std::vector<NodeID>, EdgeWeight, unsigned long>>& tree);
    SizeNumNodesPair calculateSizes(NodeID root, std::vector<std::tuple<std::vector<NodeID>, EdgeWeight, unsigned long, unsigned>>& tree, std::vector<NodeID>& landmarks);
    NodeID getMaxLeaf(NodeID root, std::vector<std::tuple<std::vector<NodeID>, EdgeWeight, unsigned long, unsigned>>& tree);
    std::vector<NodeID> getLandmarks() {
        return landmarks;
    }
private:
    friend class boost::serialization::access;

    ALTParameters parameters;
    std::string networkName;
    unsigned int numNodes;
    unsigned int numEdges;
    unsigned int numLandmarks;
    std::vector<NodeID> landmarks;
    std::vector<int> vertexFromLandmarkDistances;
    ObjectList objectList;

    // Non-Serialized Members

    // state for candidate generation
    BinaryMinHeap<EdgeWeight, NodeID> pqueue;
    int queryClosestLandmark;
    int queryLandmarkDistance;
    int candidateLeftIndex;
    int candidateRightIndex;

    void updateCandidatesQueue(NodeID q);

    std::vector<unsigned> selectBestLandmarks(NodeID s, NodeID t);

    inline EdgeWeight nodeFromLandmarkDistance(unsigned landmarkIndex, NodeID node)
    {
        return vertexFromLandmarkDistances[node * numLandmarks + landmarkIndex];
    }

    std::set<int> edgesAccessed;

    inline EdgeWeight getEdgeWeight(Graph &graph, int i)
    {
//        edgesAccessed.insert(i);
        return graph.edges[i].second;
    }

    // Boost Serialization
    template<class Archive>
    void serialize(Archive &ar, const unsigned int version)
    {
        ar & networkName;
        ar & numNodes;
        ar & numEdges;
        ar & numLandmarks;
        ar & landmarks;
        ar & vertexFromLandmarkDistances;
    }
};


#endif // _ALT_H