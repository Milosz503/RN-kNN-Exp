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

#ifndef _INE_H
#define _INE_H

#include "DynamicGraph.h"
#include "Graph.h"
#include "../common.h"
#include "../utility/Statistics.h"

#include <vector>
#include <set>

class INE {

public:
    void getKNNsByDynamicGraph(DynamicGraph &graph, unsigned int k, NodeID queryNodeID, std::vector<NodeID> &kNNs,
                               std::vector<EdgeWeight> &kNNDistances);

    void getKNNs(Graph &graph, unsigned int k, NodeID queryNodeID, std::vector<NodeID> &kNNs,
                 std::vector<EdgeWeight> &kNNDistances);

    Statistics stats;

    unsigned long edgesAccessedCount = 0;
    unsigned long distanceSum = 0;

    inline EdgeWeight getEdgeWeight(Graph &graph, int i)
    {
//        edgesAccessed.insert(i);
        return graph.edges[i].second;
    }

private:
    std::set<int> edgesAccessed;
};

#endif // _INE_H