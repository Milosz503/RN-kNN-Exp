//
// Created by milos on 22/03/2024.
//

#ifndef ND_KNN_KALT_H
#define ND_KNN_KALT_H


#include <vector>
#include "../../common.h"
#include "../Graph.h"
#include "../ALT.h"
#include "../StaticRtree.h"

class KALT {
public:

    void build(Graph& graph, unsigned int numLandmarks);

    void getKNNs(unsigned int k, NodeID queryNodeID, std::vector<NodeID>& kNNs,
                 std::vector<EdgeWeight>& kNNDistances, Graph& graph);

private:
    ALT alt;
    StaticRtree* rtree;
};


#endif //ND_KNN_KALT_H
