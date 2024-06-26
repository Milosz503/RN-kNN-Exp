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

#include "Graph.h"

#include "../utility/geometry.h"

#include <deque>
#include <fstream>
#include <iostream>
#include <sstream>
#include <regex>
#include <assert.h>
#include <unordered_map>
#include <cmath>

int Graph::getNumNodes() {
    return this->numNodes;
}

int Graph::getNumEdges()
{
    return this->numEdges;
}

std::string Graph::getNetworkName()
{
    return this->networkName;
}

std::string Graph::getEdgeType()
{
    return this->type;
}

void Graph::setGraphData(std::string name, int numNodes, int numEdges, std::string edgeType)
{
    this->networkName = name;
    this->numNodes = numNodes;
    this->numEdges = numEdges;
    this->type = edgeType;
}

void Graph::insertNode(NodeID nodeID, Coordinate x, Coordinate y)
{
    assert(nodeID == this->coordinates.size() && "Node not inserted in order");
    
    this->coordinates.push_back(CoordinatePair(x,y));
    
    // Store the minimum and maximum X and Y coordinates values
    // This is needed for quadtree contruction of this graph
    if (nodeID == 0) {
        // This is the first node, so we need to initialize min/max coordinates
        this->minXCoord = x;
        this->maxXCoord = x;
        this->minYCoord = y;
        this->maxYCoord = y;
        this->minXNode = nodeID;
        this->maxXNode = nodeID;
        this->minYNode = nodeID;
        this->maxYNode = nodeID;
    } else {
        if (x < this->minXCoord) {
            this->minXCoord = x;
            this->minXNode = nodeID;
        }
        if (x > this->maxXCoord) {
            this->maxXCoord = x;
            this->maxXNode = nodeID;
        }
        if (y < this->minYCoord) {
            this->minYCoord = y;
            this->minYNode = nodeID;
        }
        if (y > this->maxYCoord) {
            this->maxYCoord = y;
            this->maxYNode = nodeID;
        }
    }
}

// We assume edges are added in order (i.e. all edges for sourceID are added together
void Graph::insertEdge(NodeID sourceID, NodeID targetID, EdgeWeight edgeWeight) {
    assert(sourceID != targetID && "Self-loop detected");
    if (this->type == constants::DISTANCE_WEIGHTS) {
        EdgeWeight euclidDist = std::ceil(this->getEuclideanDistance(sourceID,targetID));
        if (edgeWeight < euclidDist) {
            std::cout << this->networkName << std::endl;
            std::cout << sourceID << std::endl;
            std::cout << targetID << std::endl;
            std::cout << edgeWeight << std::endl;
            std::cout << euclidDist << std::endl;
        }
        assert(edgeWeight >= euclidDist && "Edge weight smaller than Euclidean distance between neighbours");
    } else if (this->type == constants::TIME_WEIGHTS) {
        EuclideanDistance euclidDist = this->getEuclideanDistance(sourceID,targetID);
        double edgeSpeed = euclidDist/edgeWeight;
        if (edgeSpeed > this->maxGraphSpeedByEdge) {
            this->maxGraphSpeedByEdge = edgeSpeed;
        }
        if (edgeSpeed < this->minGraphSpeedByEdge) {
            this->minGraphSpeedByEdge = edgeSpeed;
        }
    }
    assert(edgeWeight != 0 && "Zero-edge weight detected");
    
    if (sourceID == this->firstEdgeIndex.size()) {
        // Then this is the first edge encountered
        this->firstEdgeIndex.push_back(this->edges.size());
    } else {
        assert(sourceID == this->firstEdgeIndex.size()-1 && "Edge not inserted in order");
        // Check if edge has already been
        int adjListStart = this->firstEdgeIndex[sourceID], adjListSize = this->edges.size();
        for (int i = adjListStart; i < adjListSize; ++i) {
            assert(targetID != this->edges[i].first && "Parallel edge detected");
        }
    }
    this->edges.push_back(NodeEdgeWeightPair(targetID,edgeWeight));
}

void Graph::parseGraphFile(std::string grFilePath, std::string coFilePath)
{
    NodeID sourceID, targetID;
    EdgeWeight weight;
    std::string name, type;
    unsigned int nodes, edges;
    Coordinate x, y;

    // Parse Coordinates File
    // We expect all coordinates to be numbered from 0 to n-1 and are supplied in order
    std::ifstream coordFile(coFilePath, std::ios::in);
    if (coordFile.is_open()) {
        coordFile >> name >> type >> nodes;
        this->networkName = name;
        this->numNodes = nodes;
            
        std::cout << "Now parsing " << nodes << " nodes for graph " << this->networkName << std::endl;

        while (coordFile >> sourceID >> x >> y)  {
           this->insertNode(sourceID,x,y);
        }
    } else {
        std::cerr << "Cannot open coordinates file " << coFilePath << std::endl;
        exit(1);
    }    
    coordFile.close();
    
    // Parse Graph File
    // We expect all edges to be supplied in order of the source ID (from 0 to n-1)
    std::ifstream graphFS(grFilePath, std::ios::in);
    if (graphFS.is_open()) {
        // First line is graph information
        graphFS >> name >> type >> nodes >> edges;
        this->numEdges = edges;
        this->type = type; // Edges file sets type not coordinates
            
        std::cout << "Now parsing " << edges << " edges for graph " << this->networkName << std::endl;

        while (graphFS >> sourceID >> targetID >> weight)  {
            this->insertEdge(sourceID,targetID,weight);
        }
        // The last element firstEdgeIndex is the size of the edges vector
        this->firstEdgeIndex.push_back(this->edges.size());
    } else {
        std::cerr << "Cannot open graph file " << grFilePath << std::endl;
        exit(1);
    }
    graphFS.close();
    
    std::cout << "\nSuccessfully parsed graph " << this->networkName << " with " << this->numNodes
                << " nodes and " << this->numEdges << " edges!" << std::endl;

    assert(this->edges.size() == static_cast<unsigned int>(this->getNumEdges()) && "Number of edges added not equal to expected number");
    assert(this->firstEdgeIndex.size() == static_cast<unsigned int>(this->getNumNodes()+1) && "Number of nodes added not equal to expected number");
    assert(this->coordinates.size() == static_cast<unsigned int>(this->getNumNodes()) && "Number of coordinate pairs added not equal to expected number");
}

std::vector<NodeID> Graph::getNodesIDsVector()
{
    std::vector<NodeID> nodeIDs;
    for (NodeID i = 0; i < this->numNodes; ++i) {
        nodeIDs.push_back(i);
    }
    return nodeIDs;
}

std::unordered_set<NodeID> Graph::getNodesIDsUset()
{
    std::unordered_set<NodeID> nodeIDs;
    for (NodeID i = 0; i < this->numNodes; ++i) {
        nodeIDs.insert(i);
    }
    return nodeIDs;
}

Coordinate Graph::getMaxCoordinateRange()
{
    // Return the largest range in coordinate values
    Coordinate xRange = this->maxXCoord-this->minXCoord, yRange = this->maxYCoord-this->minYCoord;
    return (xRange > yRange) ? xRange : yRange;
}
void Graph::getMinMaxCoordinates(Coordinate &minX, Coordinate &maxX, Coordinate &minY, Coordinate &maxY) {
    minX = this->minXCoord;
    maxX = this->maxXCoord;
    minY = this->minYCoord;
    maxY = this->maxYCoord;
}

// Position is the offset in of the neighbour node in u's adjacency list
EdgeWeight Graph::getNeighbourAndEdgeWeightByPosition(NodeID u, EdgeID pos, EdgeWeight& edgeWeight) {
    NodeID edgeIndex = this->firstEdgeIndex[u]+pos;
    edgeWeight = this->edges[edgeIndex].second;
    return this->edges[edgeIndex].first;
}

EdgeWeight Graph::getNeighbourAndEdgeWeightByPosition(NodeID u, EdgeID pos, EdgeWeight& edgeWeight, NodeID& edgeIndex) {
    edgeIndex = this->firstEdgeIndex[u]+pos;
    edgeWeight = this->edges[edgeIndex].second;
    return this->edges[edgeIndex].first;
}

void Graph::getCoordinates(NodeID u, Coordinate &x, Coordinate &y) {
    x = this->coordinates[u].first;
    y = this->coordinates[u].second;
}

void Graph::getTranslatedCoordinates(NodeID u, Coordinate &x, Coordinate &y, int xTranslation, int yTranslation) {
    // xTranslation and yTranslation represent the translation of 
    // the origin to keep all x and y values positive
    x = this->coordinates[u].first;
    y = this->coordinates[u].second;
    x = x - xTranslation;
    y = y - yTranslation;
}

EuclideanDistance Graph::getEuclideanDistance(NodeID node1, NodeID node2)
{
    return geometry::getEuclideanDist(this->coordinates[node1].first,this->coordinates[node1].second,
                                      this->coordinates[node2].first,this->coordinates[node2].second);
}

int Graph::getEdgeListStartIndex(NodeID node)
{
    return this->firstEdgeIndex[node];
}

int Graph::getEdgeListSize(NodeID node)
{
    // Since firstEdgeIndex has numNodes+1
    // element as long as node is a valid node
    // (i.e. less than numNodes) then this
    // will access a valid element of firstEdgeIndex
    return this->firstEdgeIndex[node+1];
}

/*
 * Graph Validation/Statistics Functions
 */

// Is node2 one of node1's neighbours
bool Graph::isConnected(NodeID node1, NodeID node2)
{
    int adjListStart, nextAdjListStart;
    adjListStart = this->getEdgeListStartIndex(node1);
    nextAdjListStart = this->getEdgeListSize(node1);
    // Note: We have to make sure we don't exceed size of graph.edges vector
    
    // Iterate over each neighbour of node1
    for (int i = adjListStart; i < nextAdjListStart; ++i) {
        if (this->edges[i].first == node2) {
            return true;
        }
    }
    // Could not find node2 in neighbour list
    return false;
}

bool Graph::isUndirectedGraph()
{
    bool undirected = true;
    int adjListStart, nextAdjListStart;
    for (NodeID i = 0; i < this->numNodes; ++i) {
        adjListStart = this->getEdgeListStartIndex(i);
        nextAdjListStart = this->getEdgeListSize(i);
        // Note: We have to make sure we don't exceed size of graph.edges vector
        
        // Iterate over each neighbour of current node i
        for (int j = adjListStart; j < nextAdjListStart; ++j) {
            if (!this->isConnected(this->edges[j].first,i)) {
                // If the reverse edge doesn't exist then graph is directed
                //std::cerr << "Edge between " << i << " and " << this->edges[j].first << " is directed!" << std::endl;
                undirected = false;
            }
        }
    }
    return undirected;
}

bool Graph::isConnectedGraph()
{
    bool connected = true;
    for (NodeID i = 0; i < this->numNodes; ++i) {
        if (this->firstEdgeIndex[i] == this->firstEdgeIndex[i+1]) {
            connected = false;
            std::cerr << "Node " << i << " has no edges!" << std::endl;
        }
    }
    return connected;
}

bool Graph::isContiguous()
{
    NodeID currentNode, neighbourNodeID;
    int adjListStart, nextAdjListStart;
    std::vector<bool> isNodeFound(this->numNodes,false);
    
    unsigned int regionSize = 0;
    std::deque<NodeID> pqueue;
    pqueue.push_back(0);
    while (pqueue.size() > 0) {
        currentNode = pqueue.front();
        pqueue.pop_front();
        if (!isNodeFound[currentNode]) {
            isNodeFound[currentNode] = true;
            ++regionSize;

            // Inspect each neighbour and update pqueue using edge weights
            adjListStart = this->getEdgeListStartIndex(currentNode);
            nextAdjListStart = this->getEdgeListSize(currentNode);

            for (int i = adjListStart; i < nextAdjListStart; ++i) {
                neighbourNodeID = this->edges[i].first;
                // Only add those we haven't already seen
                if (!isNodeFound[neighbourNodeID]) {
                    pqueue.push_back(neighbourNodeID);
                }
            }

        }
    }
    
    if (regionSize == this->numNodes) {
        return true;
    } else {
        return false;
    }
}

std::vector<int> Graph::countOutdegreeFrequencies()
{
    std::vector<int> frequencies;
    unsigned int outdegree;
    int numCountedEdges = 0;
    for (NodeID i = 0; i < this->numNodes; ++i) {
        // Outdegree for node i-1 (not node i)
        outdegree = this->firstEdgeIndex[i+1]-firstEdgeIndex[i];
        if (frequencies.size() <= outdegree) {
            // Note: resize doesn't change the values
            // of existing values (unlike assign)
            frequencies.resize(outdegree+1,0);
        }
        ++frequencies[outdegree];
        numCountedEdges += outdegree;
    }
    assert(numCountedEdges == this->getNumEdges() && "Frequency did not count all edges");
    return frequencies;
}

void Graph::printGraphStats()
{
    std::cout << "\nStatistics for Network " << this->networkName << ":" << std::endl;
    std::cout << "Edge Type = " << this->type << ", Nodes = " <<this->numNodes << ", Edges = " << this->numEdges << std::endl;
    std::cout << "Max Coordinate Range = " << this->getMaxCoordinateRange() << std::endl;
    std::cout << "Max Graph Speed by Edges = " << this->maxGraphSpeedByEdge << std::endl;
    std::cout << "Min Graph Speed by Edges = " << this->minGraphSpeedByEdge << std::endl;
    std::vector<int> outdegreeFrequencies = this->countOutdegreeFrequencies();
    std::cout << "\nOutdegree Frequencies:" << std::endl;
    for (std::size_t i = 0; i < outdegreeFrequencies.size(); ++i) {
        std::cout << "Outdegree of " << i << ": " << outdegreeFrequencies[i] << std::endl;
    }
    std::cout << "\nConnected = " << (this->isConnectedGraph() ? "Yes" : "No") << std::endl;
    std::cout << "Undirected = " << (this->isUndirectedGraph() ? "Yes" : "No") << std::endl;
    std::cout << "Contiguous = " << (this->isContiguous() ? "Yes" : "No") << std::endl;
    std::cout << "Min (X,Y) = " << "(" << minXCoord << "," << minYCoord << ")" << std::endl;
    std::cout << "Max (X,Y) = " << "(" << maxXCoord << "," << maxYCoord << ")" << std::endl;
    std::cout << "Min X Node= " << minXNode << ", Max X Node= " << maxXNode << std::endl;
    std::cout << "Min Y Node= " << minYNode << ", Max Y Node= " << maxYNode << std::endl;
}

// Find the end of road given in the direction of the edge provided
NodeID Graph::findRoadEnd(NodeID sourceNode, NodeID edgeIndex, int& outDegree, EdgeWeight& distanceToEnd)
{
    NodeID currNode, nextNode, prevNode, adjNode;
    int adjListStart, nextAdjListStart;
    
    currNode = this->edges[edgeIndex].first;
    distanceToEnd = this->edges[edgeIndex].second;
    prevNode = sourceNode;
    nextNode = currNode; 
    // Note: nextNode is guaranteed to be overwritten because 
    // outdegree is 2 (excluding prevNode there is at
    // least one more node)
    while (true) {
        adjListStart = this->getEdgeListStartIndex(currNode);
        nextAdjListStart = this->getEdgeListSize(currNode);
        outDegree = nextAdjListStart - adjListStart;
        if (outDegree == 2) {
            for (int i = adjListStart; i < nextAdjListStart; ++i) {
                // Only traverse away from the source node we came from
                // i.e. don't take the edge back to the prev node
                adjNode = this->edges[i].first;
                if (adjNode != prevNode) {
                    nextNode = adjNode;
                    distanceToEnd += this->edges[i].second;
                }
            }
            //assert(currNode != nextNode);
            prevNode = currNode;
            currNode = nextNode;
        } else {
            break;
        }
    }
    // Note: outDegree will be set to the outDegree of currNode
    return currNode;
}

double Graph::getMaxGraphSpeedByEdge()
{
    return this->maxGraphSpeedByEdge;
}

double Graph::getMinGraphSpeedByEdge()
{
    return this->minGraphSpeedByEdge;
}

void Graph::outputToTSVFile(std::string outputFilePath)
{
    int adjListStart, nextAdjListStart, euclideanDistance;
    std::ofstream outputTSVFile(outputFilePath, std::ios::out | std::ios::trunc);
    if (outputTSVFile.is_open()) {
        for (NodeID node = 0; node < this->numNodes; ++node) {
            adjListStart = this->getEdgeListStartIndex(node);
            nextAdjListStart = this->getEdgeListSize(node);

            for (int j = adjListStart; j < nextAdjListStart; ++j) {
                // Note: We lower bound above so that it never exceeds possible distance edge weight
                if (this->edges[j].first > node) {
                    // Only output neighbours that we have not seen yet, so that TSV file is undirected
//                     if (this->type == constants::DISTANCE_WEIGHTS) {
//                         outputTSVFile << node << "\t" << this->edges[j].first << "\t" << this->edges[j].second << "\t" << this->edges[j].second << "\n";
//                     } else {
//                         assert(this->type == constants::TIME_WEIGHTS);
                        euclideanDistance = std::floor(this->getEuclideanDistance(node,this->edges[j].first));
                        outputTSVFile << node << "\t" << this->edges[j].first << "\t" << this->edges[j].second << "\t" << euclideanDistance << "\n";
//                     }
                }
            }
        }
    } else {
        std::cerr << "Cannot open output TSV file " << outputFilePath << std::endl;
    }
    outputTSVFile.close();
}

void Graph::outputToDDSGFile(std::string outputFilePath)
{
    std::ofstream outputGrFile(outputFilePath, std::ios::out | std::ios::trunc);
    if (outputGrFile.is_open()) {
        if (!this->isUndirectedGraph()) {
            std::cerr << "Graph does not fully support undirected graph yet, cannot create DDSG file" << std::endl;
        }
        outputGrFile << this->numNodes << " " << this->numEdges << "\n";
        int adjListStart, nextAdjListStart;
        for (NodeID i = 0; i < this->numNodes; ++i) {
            adjListStart = this->getEdgeListStartIndex(i);
            nextAdjListStart = this->getEdgeListSize(i);
            for (int j = adjListStart; j < nextAdjListStart; ++j) {
                outputGrFile << i << " " << this->edges[j].first << " " << this->edges[j].second << "\n";;
                //assert(this->isConnected(this->edges[j].first,i) && "Edge is bidirectional!");
                // Note: This means directed graphs cannot be output by this function. This is because
                // DDSG files require the last value in the value to be 3 if the edge is only valid in
                // the reverse direction. To support this Graph must be modified to cover incident
                // edges to a node as well as outgoing edges.
            }
        }
    } else {
        std::cerr << "Cannot open graph output file " << outputFilePath << std::endl;
    }
    outputGrFile.close();
}

void Graph::outputZeroedCoordinatesToFile(std::string outputFilePath)
{
    std::ofstream outputCoordFile(outputFilePath, std::ios::out | std::ios::trunc);
    if (outputCoordFile.is_open()) {
        outputCoordFile << this->numNodes << "\n";
        for (NodeID i = 0; i < this->coordinates.size(); ++i) {
            outputCoordFile << i << " " << this->coordinates[i].first - this->minXCoord << " " << this->coordinates[i].second - this->minYCoord << "\n";
        }
    } else {
        std::cerr << "Cannot open coordinate output file " << outputFilePath << std::endl;
    }
    outputCoordFile.close();
}

void Graph::outputDIMACSFiles(std::string grFilePath, std::string coFilePath)
{
    std::ofstream outputCoordFile(coFilePath, std::ios::out | std::ios::trunc);
    if (outputCoordFile.is_open()) {
        outputCoordFile << "c Graph Output in Format from 9th DIMACS Implementation Challenge: Shortest Paths" << "\n";
        outputCoordFile << "c Reference: http://www.dis.uniroma1.it/~challenge9" << "\n";
        outputCoordFile << "c Graph nodes coords for " << this->networkName << "\n";
        outputCoordFile << "c" << "\n";
        outputCoordFile << "p aux sp co " << this->numNodes << "\n";
        outputCoordFile << "c Graph contains " << this->numNodes << " nodes" << "\n";
        outputCoordFile << "c" << "\n";
        for (NodeID i = 0; i < this->coordinates.size(); ++i) {
            // Note: DIMACS format starts counting nodes from 1 so we offset all nodes by 1
            outputCoordFile << "v " << i+1 << " " << this->coordinates[i].first << " " << this->coordinates[i].second << "\n";
        }
    } else {
        std::cerr << "Cannot open coordinate output file " << coFilePath << std::endl;
    }
    outputCoordFile.close();
    
    std::ofstream outputGrFile(grFilePath, std::ios::out | std::ios::trunc);
    if (outputGrFile.is_open()) {
        outputGrFile << "c Graph Output in Format from 9th DIMACS Implementation Challenge: Shortest Paths" << "\n";
        outputGrFile << "c Reference: http://www.dis.uniroma1.it/~challenge9" << "\n";
        outputGrFile << "c Graph nodes coords for " << this->networkName << "\n";
        outputGrFile << "c" << "\n";
        outputGrFile << "p sp " << this->numNodes << " " << this->numEdges << "\n";
        std::string edgeType = (this->type == constants::DISTANCE_WEIGHTS ? "distance" : "travel-time");
        outputGrFile << "c Graph contains " << this->numNodes << " nodes and " << this->numEdges << " representing " << edgeType << "\n";
        outputGrFile << "c" << "\n";

        int adjListStart, nextAdjListStart;
        for (NodeID i = 0; i < this->numNodes; ++i) {
            adjListStart = this->getEdgeListStartIndex(i);
            nextAdjListStart = this->getEdgeListSize(i);
            for (int j = adjListStart; j < nextAdjListStart; ++j) {
                // Note: DIMACS format starts counting nodes from 1 so we offset all nodes by 1
                outputGrFile << "a " << i+1 << " " << this->edges[j].first+1 << " " << this->edges[j].second << "\n";
            }
        }
    } else {
        std::cerr << "Cannot open graph output file " << grFilePath << std::endl;
    }
    outputGrFile.close();
}

Graph Graph::createSubGraph(std::string subgraphName, std::unordered_set<NodeID>& subgraphNodes)
{
    int adjListStart, nextAdjListStart, numEdges = 0;
    
    // Remove any nodes that don't have edges to other subgraph nodes
    std::unordered_set<NodeID> tempSubgraphNodes;
    tempSubgraphNodes.reserve(subgraphNodes.size());
    int edgesInSubgraph;
    for (auto it = subgraphNodes.begin(); it != subgraphNodes.end(); ++it) {
        adjListStart = this->getEdgeListStartIndex(*it);
        nextAdjListStart = this->getEdgeListSize(*it);
        edgesInSubgraph = 0;
        for (int j = adjListStart; j < nextAdjListStart; ++j) {
            // Only insert edges to node within subgraph
            if (subgraphNodes.find(this->edges[j].first) != subgraphNodes.end()) {
                ++edgesInSubgraph;
            }
        }
        if (edgesInSubgraph != 0) {
            tempSubgraphNodes.insert(*it);
        }
    }    
    subgraphNodes.swap(tempSubgraphNodes);
    tempSubgraphNodes.clear();
    
    // Creat Graph object containing only subgraph nodes provided
    std::vector<NodeID> newNodeOrder;
    std::unordered_map<NodeID,NodeID> oldIDtoNewIDMap;
    newNodeOrder.reserve(subgraphNodes.size());
    oldIDtoNewIDMap.reserve(subgraphNodes.size());
    
    for (auto it = subgraphNodes.begin(); it != subgraphNodes.end(); ++it) {
        oldIDtoNewIDMap[*it] = newNodeOrder.size();
        newNodeOrder.push_back(*it);
    }
    
    Graph subGraph;
    // Insert Nodes
    for (NodeID node = 0; node < newNodeOrder.size(); ++node) {
        subGraph.insertNode(node,this->coordinates[newNodeOrder[node]].first,this->coordinates[newNodeOrder[node]].second);
    }
    
    // Insert Edges
    for (NodeID node = 0; node < newNodeOrder.size(); ++node) {
        adjListStart = this->getEdgeListStartIndex(newNodeOrder[node]);
        nextAdjListStart = this->getEdgeListSize(newNodeOrder[node]);
        for (int j = adjListStart; j < nextAdjListStart; ++j) {
            // Only insert edges to node within subgraph
            if (subgraphNodes.find(this->edges[j].first) != subgraphNodes.end()) {
                ++numEdges;
                subGraph.insertEdge(node,oldIDtoNewIDMap[this->edges[j].first],this->edges[j].second);
            }
        }
    }
    // Need to mark end fo edges index so we don't buffer overflow
    subGraph.firstEdgeIndex.push_back(subGraph.edges.size());
    
    // Set Graph Data
    subGraph.setGraphData(subgraphName,newNodeOrder.size(),numEdges,this->type);
    
    assert(subGraph.isConnectedGraph() && "New subgraph is not a strongly connected graph!");
    assert(subGraph.getNumNodes()+1 == static_cast<int>(subGraph.firstEdgeIndex.size()) && "Subgraph has incorrect number of nodes");
    assert(subGraph.getNumEdges() == static_cast<int>(subGraph.edges.size()) && "Subgraph has incorrect number of edges");
    
    return subGraph;
}

/*
 * INE-Related Object Functions
 */

void Graph::parseObjectFile(std::string objectSetFilePath, std::string& setType, double& setDensity, int& setVariable, int& setSize)
{
    this->nodeObjects.assign(this->numNodes,false);
    
    NodeID objectID;
    std::ifstream objSetFS(objectSetFilePath, std::ios::in);
    std::string networkName; 
    
    if (objSetFS.is_open()) {
        // First line is object set information
        objSetFS >> networkName >> setType >> setDensity >> setVariable >> setSize;
        
        if (networkName != this->getNetworkName()) {
            std::cerr << "This object set was not generated for current graph " 
                << "(generated for " << networkName << ")!" << std::endl;
            exit(1);
        }
        
        while (objSetFS >> objectID)  {
            this->nodeObjects[objectID] = true;
        }
    } else {
        std::cerr << "Cannot open object set file " << objectSetFilePath << std::endl;
        exit(1);
    }
    
    objSetFS.close();
}

void Graph::parseObjectSet(std::vector<NodeID>& set)
{
    this->nodeObjects.assign(this->numNodes,false);
    for(std::size_t i = 0; i < set.size(); ++i) {
        this->nodeObjects[set[i]] = true;
    }
}

void Graph::resetAllObjects()
{
    this->nodeObjects.assign(this->numNodes,false);
}

bool Graph::isObject(NodeID nodeID)
{
    return this->nodeObjects[nodeID];
}

double Graph::computeIndexSize()
{
    double memoryUsageBytes = 0;
    memoryUsageBytes += sizeof(unsigned int)*this->firstEdgeIndex.size();
    memoryUsageBytes += sizeof(NodeEdgeWeightPair)*this->edges.size();
    memoryUsageBytes += sizeof(CoordinatePair)*this->coordinates.size();
    return memoryUsageBytes/(1024*1024);
}

double Graph::computeINEIndexSize()
{
    double memoryUsageBytes = 0;
    memoryUsageBytes += sizeof(unsigned int)*this->firstEdgeIndex.size();
    memoryUsageBytes += sizeof(NodeEdgeWeightPair)*this->edges.size();
    // We also include memory used to store the object set
    memoryUsageBytes += this->nodeObjects.size()/8; // std::vector<bool> only use 1 bit per element
    return memoryUsageBytes/(1024*1024);
}

double Graph::computeMemoryUsage()
{
    double memoryUsageBytes = 0;
    memoryUsageBytes += sizeof(*this);
    memoryUsageBytes += this->networkName.size();
    memoryUsageBytes += this->type.size();
    memoryUsageBytes += sizeof(unsigned int)*this->firstEdgeIndex.capacity();
    memoryUsageBytes += sizeof(NodeEdgeWeightPair)*this->edges.capacity();
    memoryUsageBytes += sizeof(CoordinatePair)*this->coordinates.capacity();
    memoryUsageBytes += this->nodeObjects.capacity()/8; // std::vector<bool> only use 1 bit per element
    return memoryUsageBytes/(1024*1024);
}
