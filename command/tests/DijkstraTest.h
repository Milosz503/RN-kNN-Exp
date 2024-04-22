//
// Created by milos on 18/04/2024.
//

#ifndef ND_KNN_DIJKSTRATEST_H
#define ND_KNN_DIJKSTRATEST_H

#include "UnitTest.h"
#include "../../utility/serialization.h"
#include "../../processing/Graph.h"
#include "../../processing/DijkstraSearch.h"
#include "../../utility/StopWatch.h"

class DijkstraTest : public UnitTest {
public:
    void run() override
    {
        srand(0);
        auto graph = serialization::getIndexFromBinaryFile<Graph>(
                "/home/milosz/Documents/AdaptiveIndexingGraph/RN-kNN-Results/indexes/NW.bin");

        runTest("path lengths should be correct", [this, &graph]() {
            DijkstraSearch dijkstra;
            NodeID source = 0;
            std::vector<EdgeWeight> distances(graph.getNumNodes(), 0);
            std::vector<unsigned> pathLengths(graph.getNumNodes(), 0);
            BinaryMinHeap<EdgeWeight, NodeData> pqueueLengths;

            dijkstra.findSSSPDistances(graph, source, distances, pathLengths, &pqueueLengths);

            std::vector<NodeID> testTargets;
            const unsigned testTargetsAmount = 50;

            for(unsigned i = 0; i < testTargetsAmount; ++i) {
                testTargets.push_back(rand() % graph.getNumNodes());
            }
            std::vector<NodeID> pathTree(graph.getNumNodes());
            for(auto target : testTargets) {
                auto result = dijkstra.findShortestPath(graph, source, target, pathTree);
                shouldBeEqual(pathLengths[target], (unsigned)result.getNumLinks());
                shouldBeEqual(distances[target], (unsigned)result.getLength());
            }
        });

        runTest("compare dijkstra times", [this, &graph]() {
            DijkstraSearch dijkstra;
            std::vector<EdgeWeight> distances(graph.getNumNodes(), 0);
            std::vector<unsigned> pathLengths(graph.getNumNodes(), 0);
            BinaryMinHeap<EdgeWeight, NodeData> pqueueLengths;
            BinaryMinHeap<EdgeWeight, NodeID> pqueue;

            std::vector<NodeID> testSources;
            const unsigned testSourcesAmount = 50;
            for(unsigned i = 0; i < testSourcesAmount; ++i) {
                testSources.push_back(rand() % graph.getNumNodes());
            }

            StopWatch sw;
            sw.start();
            for(auto source : testSources) {
                dijkstra.findSSSPDistances(graph, source, distances, pathLengths, &pqueueLengths);
            }
            sw.stop();
            std::cout << "Dijkstra with path lengths time: " << sw.getTimeMs() << std::endl;

            sw.reset();
            sw.start();
            for(auto source : testSources) {
                dijkstra.findSSSPDistances(graph, source, distances, &pqueue);
            }
            sw.stop();
            std::cout << "Dijkstra with only distances time: " << sw.getTimeMs() << std::endl;
        });
    }

};

#endif //ND_KNN_DIJKSTRATEST_H
