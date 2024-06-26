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

#ifndef _FASTGRAPH_H
#define _FASTGRAPH_H

#include "../common.h"
#include "Graph.h"

#include <vector>
#include <unordered_set>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/serialization/vector.hpp>

/*
 * Directed Graph Stored In-Memory
 */

class Node {
    public:
        Node(NodeID id, Coordinate x, Coordinate y): id(id), x(x), y(y) {}
        Node() {}
        Node(const Node& obj);
        const std::vector<NodeID>& getAdjNodes() const;
        const std::vector<EdgeWeight>& getAdjNodeWgts() const;
        EdgeWeight getEdgeWeight(NodeID v);
        NodeID getNeighbourAndEdgeWeightByPosition(EdgeID pos, EdgeWeight& edgeWeight);
        void getCoordinates(Coordinate& x, Coordinate& y);
        bool edgeExists(NodeID neighbour);
        inline void clear() {
            adjNodes.clear();
            adjNodeWgts.clear();
            invAdjNodes.clear();
            invAdjNodeWgts.clear();
        }
        double computeIndexSizeBytes();
        double computeINEIndexSizeBytes();
        double computeMemoryUsageBytes();

        NodeID id;
        Coordinate x, y;
        std::vector<NodeID> adjNodes;
        std::vector<EdgeWeight> adjNodeWgts;
        std::vector<NodeID> invAdjNodes;
        std::vector<EdgeWeight> invAdjNodeWgts;

    private:
        friend class boost::serialization::access;

        // Boost Serialization
        template<class Archive>
        void serialize(Archive & ar, const unsigned int version)
        {
            ar & this->id;
            ar & this->x;
            ar & this->y;
            ar & this->adjNodes;
            ar & this->adjNodeWgts;
            ar & this->invAdjNodes;
            ar & this->invAdjNodeWgts;
        }
};

class DynamicGraph {

    public:
        DynamicGraph() {}
        DynamicGraph(const DynamicGraph &obj);
        DynamicGraph(Graph &obj);
        void insertNode(NodeID nodeID, Coordinate x, Coordinate y);
        void insertEdge (NodeID sourceID, NodeID targetID, EdgeWeight edgeWeight);
        void insertImaginaryNonInvertibleEdge (NodeID sourceID, NodeID targetID, EdgeWeight edgeWeight);
        std::string getNetworkName();
        int getNumNodes();
        int getNumEdges();
        int countNodes();
        int countEdges();
        void parseGraphFile(std::string grFilePath, std::string coFilePath);
        void parseObjectFile(std::string objectSetFilePath, std::string& setType, double& setDensity, int& setVariable, int& setSize);
        void parseObjectSet(std::vector<NodeID>& set);
        void resetAllObjects();
        bool isObject(NodeID nodeID);
        bool isConnected(NodeID node1, NodeID node2);
        bool isUndirectedGraph();
        bool isConnectedGraph();
        EuclideanDistance getEuclideanDistance(NodeID node1, NodeID node2);
        std::vector<NodeID> getNodesIDsVector();
        std::unordered_set<NodeID> getNodesIDsUset();
        const std::vector<NodeID>& getAdjNodes(NodeID node) const;
        const std::vector<EdgeWeight>& getAdjNodeWgts(NodeID node) const;
        const std::vector<NodeID>& getInvAdjNodes(NodeID node) const;
        const std::vector<EdgeWeight>& getInvAdjNodeWgts(NodeID node) const;
        Coordinate getMaxCoordinateRange();
        void getMinMaxCoordinates(Coordinate& minX, Coordinate& maxX, Coordinate& minY, Coordinate& maxY);
        EdgeWeight getEdgeWeight(NodeID u, NodeID v);
        NodeID getNeighbourAndEdgeWeightByPosition(NodeID u, EdgeID pos, EdgeWeight& edgeWeight);
        void getCoordinates(NodeID u, Coordinate& x, Coordinate& y);
        void getTranslatedCoordinates(NodeID u, Coordinate& x, Coordinate& y, int xTranslation, int yTranslation);
        std::vector<int> countOutdegreeFrequencies();
        void printGraphStats();
        double computeIndexSize();
        double computeINEIndexSize();
        double computeMemoryUsage();
        double computeObjectSetIndexSize();

        std::vector<Node> nodes;
        std::vector<bool> nodeObjects;

    private:
        friend class boost::serialization::access;

        std::string networkName = "Unknown";
        int numNodes;
        int numEdges;
        std::string type;

        Coordinate minXCoord, maxXCoord, minYCoord, maxYCoord;
        NodeID minXNode = 0, maxXNode = 0, minYNode = 0, maxYNode = 0;

        // Boost Serialization
        template<class Archive>
        void serialize(Archive & ar, const unsigned int version)
        {
            ar & this->networkName;
            ar & this->numNodes;
            ar & this->numEdges;
            ar & this->type;
            ar & this->nodes;
            ar & this->minXCoord;
            ar & this->maxXCoord;
            ar & this->minYCoord;
            ar & this->maxYCoord;
            ar & this->minXNode;
            ar & this->maxXNode;
            ar & this->minYNode;
            ar & this->maxYNode;
        }

};

#endif // _FASTGRAPH_H
