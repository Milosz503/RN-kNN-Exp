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

#ifndef _DIJKSTRASEARCH_H
#define _DIJKSTRASEARCH_H

#include "../common.h"
#include "DynamicGraph.h"
#include "Graph.h"
#include "Path.h"
#include "../queue/MinPriorityQueueWithDK.h"
#include "../queue/MinPriorityQueue.h"
#include "../queue/BinaryMinHeap.h"

#include <vector>
#include <unordered_set>
#include <unordered_map>
#include <iostream>
#include <set>

struct NodeData {
    NodeID node;
    unsigned pathLength;
};

class DijkstraSearch {

public:

    void findSSMTDistances(DynamicGraph &graph, NodeID source,
                           std::unordered_set<NodeID> &targetSet,
                           std::unordered_map<NodeID, EdgeWeight> &results,
                           BinaryMinHeap<EdgeWeight, NodeID> *pqueue);

    Path findShortestPath(Graph &graph, NodeID source, NodeID target, std::vector<NodeID> &shortestPathTree);

    EdgeWeight findShortestPathDistance(Graph &graph, NodeID source, NodeID target);

    void findSSSPDistances(Graph &graph, NodeID source, std::vector<EdgeWeight> &targetDistances,
                           MinPriorityQueueWithDK<EdgeWeight, NodeID> *pqueue);

    void findSSSPDistances(Graph &graph, NodeID source, std::vector<EdgeWeight> &targetDistances,
                           MinPriorityQueue<EdgeWeight, NodeID> *pqueue);

    void findSSSPDistances(Graph &graph, NodeID source, std::vector<EdgeWeight> &targetDistances,
                           std::vector<EdgeWeight> &pathLengths,
                           std::vector<unsigned> &nodesVisited,
                           MinPriorityQueue<EdgeWeight, NodeData> *pqueue);

    void findSSSPDistances(Graph &graph, NodeID source, std::vector<EdgeWeight> &targetDistances,
                           std::vector<EdgeWeight> &pathLengths,
                           MinPriorityQueue<EdgeWeight, NodeData> *pqueue);

    void findSSSPDistances(Graph &graph, NodeID source, std::vector<EdgeWeight> &targetDistances);

    void findSSMTDistances(Graph &graph, NodeID source,
                           std::unordered_set<NodeID> &targetSet,
                           std::unordered_map<NodeID, EdgeWeight> &results,
                           BinaryMinHeap<EdgeWeight, NodeID> *pqueue);

    EdgeWeight
    findShortestPathDistanceSubgraph(Graph &graph, NodeID source, NodeID target, std::vector<bool> &edgeInSubgraph);

    void findSSMTDistancesSubgraph(Graph &graph, NodeID source,
                                   std::unordered_set<NodeID> &targetSet,
                                   std::unordered_map<NodeID, EdgeWeight> &results,
                                   std::vector<bool> &edgeInSubgraph);

    bool colourizeMap(Graph &graph, NodeID source, std::vector<EdgeID> &colourMap,
                      std::vector<EdgeWeight> &distances, MinPriorityQueue<EdgeWeight, NodeLinkPair> *pqueue);

    bool colourizeMap(Graph &graph, NodeID source, std::vector<EdgeID> &colourMap,
                      std::vector<EdgeWeight> &distances);

    inline EdgeWeight getEdgeWeight(Graph &graph, int i)
    {
//            edgesAccessed.insert(i);
        return graph.edges[i].second;
    }


    void printEdgeAccess()
    {
//            std::cout << "Edge access number: " << edgesAccessed.size() << std::endl;
    }

    unsigned long edgesAccessedCount = 0;

private:
    std::set<int> edgesAccessed;
};

#endif // _DIJKSTRASEARCH_H
