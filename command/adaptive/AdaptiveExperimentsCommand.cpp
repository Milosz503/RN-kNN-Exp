//
// Created by milos on 08/03/2024.
//

#include "AdaptiveExperimentsCommand.h"

#include "../../processing/DynamicGraph.h"
#include "../../processing/Gtree.h"
#include "../../processing/adaptive_gtree/AdaptiveGtree.h"
#include "../../processing/ROAD.h"
#include "../../processing/MortonList.h"
#include "../../processing/INE.h"
#include "../../processing/IER.h"
#include "../../processing/SetGenerator.h"
#include "../../processing/ALT.h"
#include "../../processing/ShortestPathWrapper.h"
#include "../../tuple/IndexTuple.h"
#include "../../tuple/ObjectIndexTuple.h"
#include "../../tuple/KnnQueryTuple.h"
#include "../../common.h"
#include "../../utility/StopWatch.h"
#include "../../utility/Statistics.h"
#include "../../utility/utility.h"
#include "../../utility/serialization.h"
#include "IERExperiment.h"
#include "../shortest_distance/QueryGenerator.h"

#include <cstdio>
#include <cmath>
#include <fstream>


static const int STATS_STEP = 100;

void write_to_csv2(const std::vector<std::vector<double>>& results, std::vector<std::string> methods, std::string path, int numOfRepeats)
{
    std::cout << "Saving results to: " << path << std::endl;

    std::ofstream file(path + "_output.csv");
    std::ofstream file_deviation(path + "_output_dev.csv");


    for (int i = 0; i < results.size(); ++i) {
        file << methods[i] << ",";
    }
    file << std::endl;

    std::vector<std::vector<double>> avgResults;
    std::vector<std::vector<double>> stdResults;

    unsigned maxNumMeasurements = 0;

    for (auto i = 0; i < results.size(); i++) {
        int numMeasurements = results[i].size() / numOfRepeats;
        if(numMeasurements > maxNumMeasurements) {
            maxNumMeasurements = numMeasurements;
        }
        avgResults.push_back(std::vector<double>(numMeasurements));
        stdResults.push_back(std::vector<double>(numMeasurements));
        for (int j = 0; j < numMeasurements; j++) {
            double avg = 0.0;
            for (int k = 0; k < numOfRepeats; k++) {
                avg += results[i][k * numMeasurements + j];
            }
            avg /= numOfRepeats;
            double stdder = 0.0;
            for (int k = 0; k < numOfRepeats; k++) {
                stdder += std::pow((results[i][k * numMeasurements + j] - avg), 2);
            }
            stdder = std::pow((stdder / (numOfRepeats - 1)), 0.5);
            avgResults[i][j] = avg;
            stdResults[i][j] = stdder;
        }
    }

    for (int j = 0; j < maxNumMeasurements; j++) {
        for (auto i = 0; i < results.size(); i++) {
            if(j < avgResults[i].size()) {
                file << avgResults[i][j];
            }
            file << ",";

            if(j < stdResults[i].size()) {
                file_deviation << stdResults[i][j];
            }
            file_deviation << ",";
        }
        file << std::endl;
        file_deviation << std::endl;
    }

    file.close();
    file_deviation.close();
}

void experimentCompareKnn(Experiment &experiment, Graph &graph, std::string output, unsigned r = 5, int exp = 0, unsigned q = 8192, unsigned k = 10, double density = 0.001)
{
    std::vector<std::vector<double>> results(1);
    StopWatch sw;
    std::vector<NodeID> kNNs;
    std::vector<EdgeWeight> kNNDistances;
    unsigned queryCounter;
    double totalQueryTime;

    for (int repeats = 0; repeats < r; repeats++) {
        queryCounter = 0;
        totalQueryTime = 0.0;
        std::vector <NodeID> objects = QueryGenerator().randomObjects(graph, density);
        std::vector <NodeID> queries = QueryGenerator().randomKNN(graph, q);
        if (exp == 2) {
            graph.resetAllObjects();
            graph.parseObjectSet(objects);
        }
        sw.reset();
        sw.start();
        experiment.buildIndex(graph);
        sw.stop();
        results[0].push_back(sw.getTimeMs());
        experiment.printInfo();
        experiment.clearObjects();
        sw.reset();
        sw.start();
        experiment.loadObjects(graph, objects);
        sw.stop();
        results[0].push_back(sw.getTimeMs());
        for (auto queryNodeIt = queries.begin(); queryNodeIt != queries.end(); ++queryNodeIt) {
            kNNs.clear();
            kNNDistances.clear();
            kNNs.reserve(k);
            kNNDistances.reserve(k);
            sw.reset();
            sw.start();
            experiment.runQuery(graph, k, *queryNodeIt, kNNs, kNNDistances);
            sw.stop();

            totalQueryTime += sw.getTimeMs();

            queryCounter++;
            if (queryCounter == 4 || queryCounter == 16 || queryCounter == 64 ||
                queryCounter == 256 || queryCounter == 1024 || queryCounter == 4196 || queryCounter == 8192) {
                results[0].push_back(totalQueryTime);
                std::cout << "Q: " << queryCounter << ", t: " << totalQueryTime << std::endl;

            }
        }
    }

    write_to_csv2(results, std::vector<std::string>({experiment.getName()}), output + "/agtree/" + graph.getNetworkName() + "_compare_knn_" + std::to_string(exp), r);
}

void experimentCompareKnnClusteredQueries(Experiment &experiment, Graph &graph, std::string output, unsigned r = 5, unsigned exp=0, unsigned q = 8192, unsigned k = 10, double density = 0.001)
{
    std::vector<std::vector<double>> results(1);
    StopWatch sw;
    std::vector<NodeID> kNNs;
    std::vector<EdgeWeight> kNNDistances;

    unsigned queryCounter;
    double totalQueryTime;

    for (int repeats = 0; repeats < r; repeats++) {
        queryCounter = 0;
        totalQueryTime = 0.0;
        std::vector <NodeID> objects = QueryGenerator().randomObjects(graph, density);
        std::vector <NodeID> queries = QueryGenerator().randomExpandKNNQueriesClustered(graph, q, 1, 100.0 / graph.getNumNodes());
        if (exp == 2) {
            graph.resetAllObjects();
            graph.parseObjectSet(objects);
        }
        sw.reset();
        sw.start();
        experiment.buildIndex(graph);
        sw.stop();
        results[0].push_back(sw.getTimeMs());
        experiment.printInfo();
        experiment.clearObjects();
        sw.reset();
        sw.start();
        experiment.loadObjects(graph, objects);
        sw.stop();
        results[0].push_back(sw.getTimeMs());
        for (auto queryNodeIt = queries.begin(); queryNodeIt != queries.end(); ++queryNodeIt) {
            kNNs.clear();
            kNNDistances.clear();
            kNNs.reserve(k);
            kNNDistances.reserve(k);
            sw.reset();
            sw.start();
            experiment.runQuery(graph, k, *queryNodeIt, kNNs, kNNDistances);
            sw.stop();
            totalQueryTime += sw.getTimeMs();

            queryCounter++;
            if (queryCounter == 4 || queryCounter == 16 || queryCounter == 64 ||
                queryCounter == 256 || queryCounter == 1024 || queryCounter == 4196 || queryCounter == 8192) {
                results[0].push_back(totalQueryTime);
                std::cout << "Q: " << queryCounter << ", t: " << totalQueryTime << std::endl;

            }
        }
    }

    write_to_csv2(results, std::vector<std::string>({experiment.getName()}), output + "/agtree/" + graph.getNetworkName() + "_compare_knn_clustered_queries_" + std::to_string(exp), r);
}

void experimentCompareKnnClusteredObjects(Experiment &experiment, Graph &graph, std::string output, unsigned r = 5, unsigned exp=0, unsigned q = 8192, unsigned k = 10, double density = 0.001)
{
    std::vector<std::vector<double>> results(1);
    StopWatch sw;
    std::vector<NodeID> kNNs;
    std::vector<EdgeWeight> kNNDistances;

    unsigned queryCounter;
    double totalQueryTime;

    for (int repeats = 0; repeats < r; repeats++) {
        queryCounter = 0;
        totalQueryTime = 0.0;
        std::vector <NodeID> objects = QueryGenerator().randomExpandObjectsClustered(graph, 0.001, 1,
                                                                                     100.0 / graph.getNumNodes());
        std::vector <NodeID> queries = QueryGenerator().randomKNN(graph, q);
        if (exp == 2) {
            graph.resetAllObjects();
            graph.parseObjectSet(objects);
        }
        sw.reset();
        sw.start();
        experiment.buildIndex(graph);
        sw.stop();
        results[0].push_back(sw.getTimeMs());
        experiment.printInfo();
        experiment.clearObjects();
        sw.reset();
        sw.start();
        experiment.loadObjects(graph, objects);
        sw.stop();
        results[0].push_back(sw.getTimeMs());
        for (auto queryNodeIt = queries.begin(); queryNodeIt != queries.end(); ++queryNodeIt) {
            kNNs.clear();
            kNNDistances.clear();
            kNNs.reserve(k);
            kNNDistances.reserve(k);
            sw.reset();
            sw.start();
            experiment.runQuery(graph, k, *queryNodeIt, kNNs, kNNDistances);
            sw.stop();
            totalQueryTime += sw.getTimeMs();

            queryCounter++;
            if (queryCounter == 4 || queryCounter == 16 || queryCounter == 64 ||
                queryCounter == 256 || queryCounter == 1024 || queryCounter == 4196 || queryCounter == 8192) {
                results[0].push_back(totalQueryTime);
                std::cout << "Q: " << queryCounter << ", t: " << totalQueryTime << std::endl;

            }
        }
    }

    write_to_csv2(results, std::vector<std::string>({experiment.getName()}), output + "/agtree/" + graph.getNetworkName() + "_compare_knn_clustered_objects_" + std::to_string(exp), r);
}

void experimentCompareKnnClustered(Experiment &experiment, Graph &graph, std::string output, unsigned r = 5, unsigned exp = 0, unsigned q = 8192, unsigned k = 10, double density = 0.001)
{
    std::vector<std::vector<double>> results(1);
    StopWatch sw;
    std::vector<NodeID> kNNs, ineKNNs;
    std::vector<EdgeWeight> kNNDistances, ineKNNDistances;

    unsigned queryCounter;
    double totalQueryTime;

    for (int repeats = 0; repeats < r; repeats++) {
        queryCounter = 0;
        totalQueryTime = 0.0;
        std::vector <NodeID> objects = QueryGenerator().randomExpandObjectsClustered(graph, 0.001, 1,
                                                                                     100.0 / graph.getNumNodes());
        std::vector <NodeID> queries = QueryGenerator().randomExpandKNNQueriesClustered(graph, q, 1, 100.0 / graph.getNumNodes());
        if (exp == 2) {
            graph.resetAllObjects();
            graph.parseObjectSet(objects);
        }
        sw.reset();
        sw.start();
        experiment.buildIndex(graph);
        sw.stop();
        results[0].push_back(sw.getTimeMs());
        experiment.printInfo();
        experiment.clearObjects();
        sw.reset();
        sw.start();
        experiment.loadObjects(graph, objects);
        sw.stop();
        results[0].push_back(sw.getTimeMs());
        for (auto queryNodeIt = queries.begin(); queryNodeIt != queries.end(); ++queryNodeIt) {
            kNNs.clear();
            kNNDistances.clear();
            kNNs.reserve(k);
            kNNDistances.reserve(k);
            sw.reset();
            sw.start();
            experiment.runQuery(graph, k, *queryNodeIt, kNNs, kNNDistances);
            sw.stop();
            totalQueryTime += sw.getTimeMs();

            queryCounter++;
            if (queryCounter == 4 || queryCounter == 16 || queryCounter == 64 ||
                queryCounter == 256 || queryCounter == 1024 || queryCounter == 4196 || queryCounter == 8192) {
                results[0].push_back(totalQueryTime);
                std::cout << "Q: " << queryCounter << ", t: " << totalQueryTime << std::endl;

            }
        }
    }

    write_to_csv2(results, std::vector<std::string>({experiment.getName()}), output + "/agtree/" + graph.getNetworkName() + "_compare_knn_clustered_" + std::to_string(exp), r);
}

void experimentObjectDistribution(Graph &graph, unsigned fanout = 4, unsigned maxLeafSize = 128, unsigned int r = 1, double density = 0.001, unsigned k = 10, bool verifyKNN = false, std::string output = "")
{
    AdaptiveGTreeExperiment experiment(fanout, maxLeafSize);
    INEExperiment verifyExperiment;
    std::string message;


    std::vector<std::vector<double>> results(5);
    StopWatch sw;
    std::vector<NodeID> kNNs, ineKNNs;
    std::vector<EdgeWeight> kNNDistances, ineKNNDistances;
    unsigned queryCounter = 0;
    double totalQueryTime = 0.0;
    for (int object_c = 0; object_c < 5; object_c++) {
        for (int repeats = 0; repeats < r; repeats++) {
            queryCounter = 0;
            totalQueryTime = 0.0;
            std::vector <NodeID> objects;
            std::vector <NodeID> queries = QueryGenerator().randomKNN(graph, 8192);
            if (object_c == 0) {
                objects = QueryGenerator().randomObjects(graph, density);
            } else {
                int numClusters = std::pow(2, object_c - 1);
                objects = QueryGenerator().randomExpandObjectsClustered(graph, density, numClusters,
                                                                        100.0 / graph.getNumNodes());
            }
            if (verifyKNN) {
                graph.resetAllObjects();
                graph.parseObjectSet(objects);
            }
            sw.reset();
            sw.start();
            experiment.buildIndex(graph);
            sw.stop();
            results[object_c].push_back(sw.getTimeMs());
            experiment.printInfo();
            experiment.clearObjects();
            experiment.loadObjects(graph, objects);
            for (auto queryNodeIt = queries.begin(); queryNodeIt != queries.end(); ++queryNodeIt) {
                kNNs.clear();
                kNNDistances.clear();
                kNNs.reserve(k);
                kNNDistances.reserve(k);
                sw.reset();
                sw.start();
                experiment.runQuery(graph, k, *queryNodeIt, kNNs, kNNDistances);
                sw.stop();
                totalQueryTime += sw.getTimeMs();

                if (verifyKNN) {
                    ineKNNs.clear();
                    ineKNNDistances.clear();
                    verifyExperiment.clearObjects();
                    verifyExperiment.loadObjects(graph, objects);
                    verifyExperiment.runQuery(graph, k, *queryNodeIt, ineKNNs, ineKNNDistances);
                    if (!utility::verifyKNN(ineKNNs, ineKNNDistances, kNNs, kNNDistances, false, k,
                                            message, true)) {
                        std::cout << "Verfication failed for query node " << *queryNodeIt << " with k = "
                                  << k
                                  << std::endl;
                        std::cout << "Message: " << message << std::endl;
                        exit(1);
                    }
                }
                queryCounter++;
                if (queryCounter == 4 || queryCounter == 16 || queryCounter == 64 ||
                    queryCounter == 256 || queryCounter == 1024 || queryCounter == 4196 || queryCounter == 8192) {
                    results[object_c].push_back(totalQueryTime);
                    std::cout << "Q: " << queryCounter << ", t: " << totalQueryTime << std::endl;

                }
            }
        }
    }
    write_to_csv2(results, std::vector<std::string>({"uniform", "c: 1", "c: 2", "c: 4", "c: 8"}), output + "/agtree/" + graph.getNetworkName() + "_object-distribution", r);
}

void experimentObjectDistributionNA(Graph &graph, unsigned fanout = 4, unsigned maxLeafSize = 128, unsigned int r = 1, double density = 0.001, unsigned k = 10, bool verifyKNN = false, std::string output = "")
{
    GTreeExperiment experiment(fanout, maxLeafSize);
    INEExperiment verifyExperiment;
    std::string message;


    std::vector<std::vector<double>> results(5);
    StopWatch sw;
    std::vector<NodeID> kNNs, ineKNNs;
    std::vector<EdgeWeight> kNNDistances, ineKNNDistances;
    unsigned queryCounter = 0;
    double totalQueryTime = 0.0;
    for (int object_c = 0; object_c < 5; object_c++) {
        for (int repeats = 0; repeats < r; repeats++) {
            queryCounter = 0;
            totalQueryTime = 0.0;
            std::vector <NodeID> objects;
            std::vector <NodeID> queries = QueryGenerator().randomKNN(graph, 8192);
            if (object_c == 0) {
                objects = QueryGenerator().randomObjects(graph, density);
            } else {
                int numClusters = std::pow(2, object_c - 1);
                objects = QueryGenerator().randomExpandObjectsClustered(graph, density, numClusters,
                                                                        100.0 / graph.getNumNodes());
            }
            if (verifyKNN) {
                graph.resetAllObjects();
                graph.parseObjectSet(objects);
            }
            sw.reset();
            sw.start();
            experiment.buildIndex(graph);
            sw.stop();
            results[object_c].push_back(sw.getTimeMs());
            experiment.printInfo();
            experiment.clearObjects();
            experiment.loadObjects(graph, objects);
            for (auto queryNodeIt = queries.begin(); queryNodeIt != queries.end(); ++queryNodeIt) {
                kNNs.clear();
                kNNDistances.clear();
                kNNs.reserve(k);
                kNNDistances.reserve(k);
                sw.reset();
                sw.start();
                experiment.runQuery(graph, k, *queryNodeIt, kNNs, kNNDistances);
                sw.stop();
                totalQueryTime += sw.getTimeMs();

                if (verifyKNN) {
                    ineKNNs.clear();
                    ineKNNDistances.clear();
                    verifyExperiment.clearObjects();
                    verifyExperiment.loadObjects(graph, objects);
                    verifyExperiment.runQuery(graph, k, *queryNodeIt, ineKNNs, ineKNNDistances);
                    if (!utility::verifyKNN(ineKNNs, ineKNNDistances, kNNs, kNNDistances, false, k,
                                            message, true)) {
                        std::cout << "Verfication failed for query node " << *queryNodeIt << " with k = "
                                  << k
                                  << std::endl;
                        std::cout << "Message: " << message << std::endl;
                        exit(1);
                    }
                }
                queryCounter++;
                if (queryCounter == 4 || queryCounter == 16 || queryCounter == 64 ||
                    queryCounter == 256 || queryCounter == 1024 || queryCounter == 4196 || queryCounter == 8192) {
                    results[object_c].push_back(totalQueryTime);
                    std::cout << "Q: " << queryCounter << ", t: " << totalQueryTime << std::endl;

                }
            }
        }
    }
    write_to_csv2(results, std::vector<std::string>({"uniform", "c: 1", "c: 2", "c: 4", "c: 8"}), output + "/gtree/" + graph.getNetworkName() + "_object-distribution", r);
}

void experimentObjectDistributionMixed(Graph &graph, unsigned fanout = 4, unsigned maxLeafSize = 128, unsigned int r = 1, double density = 0.001, unsigned k = 10, bool verifyKNN = false, std::string output = "")
{
    AdaptiveGTreeExperiment experiment(fanout, maxLeafSize);
    INEExperiment verifyExperiment;
    std::string message;


    std::vector<std::vector<double>> results(5);
    StopWatch sw;
    std::vector<NodeID> kNNs, ineKNNs;
    std::vector<EdgeWeight> kNNDistances, ineKNNDistances;
    unsigned queryCounter = 0;
    double totalQueryTime = 0.0;
    for (int object_c = 0; object_c < 5; object_c++) {
        for (int repeats = 0; repeats < r; repeats++) {
            queryCounter = 0;
            totalQueryTime = 0.0;
            std::vector <NodeID> objects;
            std::vector <NodeID> queries = QueryGenerator().randomKNN(graph, 8192);
            if (object_c == 0) {
                objects = QueryGenerator().randomObjects(graph, 0.0005);
                auto objectsNew = QueryGenerator().randomExpandObjectsClustered(graph, 0.0005, 1, 100.0 / graph.getNumNodes());
                objects.insert(objects.end(), objectsNew.begin(), objectsNew.end());
            }
            else if (object_c == 1)
            {
                objects = QueryGenerator().randomObjects(graph, 0.00033);
                auto objectsNew = QueryGenerator().randomExpandObjectsClustered(graph, 0.00067, 1, 100.0 / graph.getNumNodes());
                objects.insert(objects.end(), objectsNew.begin(), objectsNew.end());
            }
            else if (object_c == 2)
            {
                objects = QueryGenerator().randomObjects(graph, 0.0005);
                auto objectsNew = QueryGenerator().randomExpandObjectsClustered(graph, 0.0005, 2, 100.0 / graph.getNumNodes());
                objects.insert(objects.end(), objectsNew.begin(), objectsNew.end());
            }
            else if (object_c == 3)
            {
                objects = QueryGenerator().randomObjects(graph, 0.00033);
                auto objectsNew = QueryGenerator().randomExpandObjectsClustered(graph, 0.00067, 1, 100.0 / graph.getNumNodes());
                objects.insert(objects.end(), objectsNew.begin(), objectsNew.end());
            }
            else if (object_c == 4)
            {
                objects = QueryGenerator().randomObjects(graph, 0.00025);
                auto objectsNew = QueryGenerator().randomExpandObjectsClustered(graph, 0.00075, 1, 100.0 / graph.getNumNodes());
                objects.insert(objects.end(), objectsNew.begin(), objectsNew.end());
            }
            if (verifyKNN) {
                graph.resetAllObjects();
                graph.parseObjectSet(objects);
            }
            sw.reset();
            sw.start();
            experiment.buildIndex(graph);
            sw.stop();
            results[object_c].push_back(sw.getTimeMs());
            experiment.printInfo();
            experiment.clearObjects();
            experiment.loadObjects(graph, objects);
            for (auto queryNodeIt = queries.begin(); queryNodeIt != queries.end(); ++queryNodeIt) {
                kNNs.clear();
                kNNDistances.clear();
                kNNs.reserve(k);
                kNNDistances.reserve(k);
                sw.reset();
                sw.start();
                experiment.runQuery(graph, k, *queryNodeIt, kNNs, kNNDistances);
                sw.stop();
                totalQueryTime += sw.getTimeMs();

                if (verifyKNN) {
                    ineKNNs.clear();
                    ineKNNDistances.clear();
                    verifyExperiment.clearObjects();
                    verifyExperiment.loadObjects(graph, objects);
                    verifyExperiment.runQuery(graph, k, *queryNodeIt, ineKNNs, ineKNNDistances);
                    if (!utility::verifyKNN(ineKNNs, ineKNNDistances, kNNs, kNNDistances, false, k,
                                            message, true)) {
                        std::cout << "Verfication failed for query node " << *queryNodeIt << " with k = "
                                  << k
                                  << std::endl;
                        std::cout << "Message: " << message << std::endl;
                        exit(1);
                    }
                }
                queryCounter++;
                if (queryCounter == 4 || queryCounter == 16 || queryCounter == 64 ||
                    queryCounter == 256 || queryCounter == 1024 || queryCounter == 4196 || queryCounter == 8192) {
                    results[object_c].push_back(totalQueryTime);
                    std::cout << "Q: " << queryCounter << ", t: " << totalQueryTime << std::endl;

                }
            }
        }
    }
    write_to_csv2(results, std::vector<std::string>({"c=1, q_c=4096", "c=1, q_c=6144", "c=2, q_c=4096", "c=2, q_c=6144", "c=2, q_c=7168"}), output + "/agtree/" + graph.getNetworkName() + "_object-distribution_mixed", r);
}

void experimentQueryDistribution(Graph &graph, unsigned fanout = 4, unsigned maxLeafSize = 128, unsigned int r = 1, double density = 0.001, unsigned k = 10, bool verifyKNN = false, std::string output = "")
{
    AdaptiveGTreeExperiment experiment(fanout, maxLeafSize);
    INEExperiment verifyExperiment;
    std::string message;


    std::vector<std::vector<double>> results(5);
    StopWatch sw;
    std::vector<NodeID> kNNs, ineKNNs;
    std::vector<EdgeWeight> kNNDistances, ineKNNDistances;
    unsigned queryCounter = 0;
    double totalQueryTime = 0.0;
    for (int query_c = 0; query_c < 5; query_c++) {
        for (int repeats = 0; repeats < r; repeats++) {
            queryCounter = 0;
            totalQueryTime = 0.0;
            std::vector <NodeID> objects = QueryGenerator().randomObjects(graph, density);;
            std::vector <NodeID> queries;
            if (query_c == 0) {
                queries = QueryGenerator().randomKNN(graph, 8192);
            } else {
                int numClusters = std::pow(2, query_c - 1);
                queries = QueryGenerator().randomExpandKNNQueriesClustered(graph, 8192, numClusters, 100.0 / graph.getNumNodes());
            }
            if (verifyKNN) {
                graph.resetAllObjects();
                graph.parseObjectSet(objects);
            }
            sw.reset();
            sw.start();
            experiment.buildIndex(graph);
            sw.stop();
            results[query_c].push_back(sw.getTimeMs());
            experiment.printInfo();
            experiment.clearObjects();
            experiment.loadObjects(graph, objects);
            for (auto queryNodeIt = queries.begin(); queryNodeIt != queries.end(); ++queryNodeIt) {
                kNNs.clear();
                kNNDistances.clear();
                kNNs.reserve(k);
                kNNDistances.reserve(k);
                sw.reset();
                sw.start();
                experiment.runQuery(graph, k, *queryNodeIt, kNNs, kNNDistances);
                sw.stop();
                totalQueryTime += sw.getTimeMs();

                if (verifyKNN) {
                    ineKNNs.clear();
                    ineKNNDistances.clear();
                    verifyExperiment.clearObjects();
                    verifyExperiment.loadObjects(graph, objects);
                    verifyExperiment.runQuery(graph, k, *queryNodeIt, ineKNNs, ineKNNDistances);
                    if (!utility::verifyKNN(ineKNNs, ineKNNDistances, kNNs, kNNDistances, false, k,
                                            message, true)) {
                        std::cout << "Verfication failed for query node " << *queryNodeIt << " with k = "
                                  << k
                                  << std::endl;
                        std::cout << "Message: " << message << std::endl;
                        exit(1);
                    }
                }
                queryCounter++;
                if (queryCounter == 4 || queryCounter == 16 || queryCounter == 64 ||
                    queryCounter == 256 || queryCounter == 1024 || queryCounter == 4196 || queryCounter == 8192) {
                    results[query_c].push_back(totalQueryTime);
                    std::cout << "Q: " << queryCounter << ", t: " << totalQueryTime << std::endl;

                }
            }
        }
    }
    write_to_csv2(results, std::vector<std::string>({"uniform", "c: 1", "c: 2", "c: 4", "c: 8"}), output + "/agtree/" + graph.getNetworkName() + "_query-distribution", r);
}

void experimentQueryDistributionNA(Graph &graph, unsigned fanout = 4, unsigned maxLeafSize = 128, unsigned int r = 1, double density = 0.001, unsigned k = 10, bool verifyKNN = false, std::string output = "")
{
    GTreeExperiment experiment(fanout, maxLeafSize);
    INEExperiment verifyExperiment;
    std::string message;


    std::vector<std::vector<double>> results(5);
    StopWatch sw;
    std::vector<NodeID> kNNs, ineKNNs;
    std::vector<EdgeWeight> kNNDistances, ineKNNDistances;
    unsigned queryCounter = 0;
    double totalQueryTime = 0.0;
    for (int query_c = 0; query_c < 5; query_c++) {
        for (int repeats = 0; repeats < r; repeats++) {
            queryCounter = 0;
            totalQueryTime = 0.0;
            std::vector <NodeID> objects = QueryGenerator().randomObjects(graph, density);;
            std::vector <NodeID> queries;
            if (query_c == 0) {
                queries = QueryGenerator().randomKNN(graph, 8192);
            } else {
                int numClusters = std::pow(2, query_c - 1);
                queries = QueryGenerator().randomExpandKNNQueriesClustered(graph, 8192, numClusters, 100.0 / graph.getNumNodes());
            }
            if (verifyKNN) {
                graph.resetAllObjects();
                graph.parseObjectSet(objects);
            }
            sw.reset();
            sw.start();
            experiment.buildIndex(graph);
            sw.stop();
            results[query_c].push_back(sw.getTimeMs());
            experiment.printInfo();
            experiment.clearObjects();
            experiment.loadObjects(graph, objects);
            for (auto queryNodeIt = queries.begin(); queryNodeIt != queries.end(); ++queryNodeIt) {
                kNNs.clear();
                kNNDistances.clear();
                kNNs.reserve(k);
                kNNDistances.reserve(k);
                sw.reset();
                sw.start();
                experiment.runQuery(graph, k, *queryNodeIt, kNNs, kNNDistances);
                sw.stop();
                totalQueryTime += sw.getTimeMs();

                if (verifyKNN) {
                    ineKNNs.clear();
                    ineKNNDistances.clear();
                    verifyExperiment.clearObjects();
                    verifyExperiment.loadObjects(graph, objects);
                    verifyExperiment.runQuery(graph, k, *queryNodeIt, ineKNNs, ineKNNDistances);
                    if (!utility::verifyKNN(ineKNNs, ineKNNDistances, kNNs, kNNDistances, false, k,
                                            message, true)) {
                        std::cout << "Verfication failed for query node " << *queryNodeIt << " with k = "
                                  << k
                                  << std::endl;
                        std::cout << "Message: " << message << std::endl;
                        exit(1);
                    }
                }
                queryCounter++;
                if (queryCounter == 4 || queryCounter == 16 || queryCounter == 64 ||
                    queryCounter == 256 || queryCounter == 1024 || queryCounter == 4196 || queryCounter == 8192) {
                    results[query_c].push_back(totalQueryTime);
                    std::cout << "Q: " << queryCounter << ", t: " << totalQueryTime << std::endl;

                }
            }
        }
    }
    write_to_csv2(results, std::vector<std::string>({"uniform", "c: 1", "c: 2", "c: 4", "c: 8"}), output + "/gtree/" + graph.getNetworkName() + "_query-distribution", r);
}

void experimentQueryDistributionMixed(Graph &graph, unsigned fanout = 4, unsigned maxLeafSize = 128, unsigned int r = 1, double density = 0.001, unsigned k = 10, bool verifyKNN = false, std::string output = "")
{
    AdaptiveGTreeExperiment experiment(fanout, maxLeafSize);
    INEExperiment verifyExperiment;
    std::string message;


    std::vector<std::vector<double>> results(5);
    StopWatch sw;
    std::vector<NodeID> kNNs, ineKNNs;
    std::vector<EdgeWeight> kNNDistances, ineKNNDistances;
    unsigned queryCounter = 0;
    double totalQueryTime = 0.0;
    for (int query_c = 0; query_c < 5; query_c++) {
        for (int repeats = 0; repeats < r; repeats++) {
            queryCounter = 0;
            totalQueryTime = 0.0;
            std::vector <NodeID> objects = QueryGenerator().randomObjects(graph, density);;
            std::vector <NodeID> queries;
            if (query_c == 0) {
                queries = QueryGenerator().randomKNN(graph, 4096);
                auto queriesNew = QueryGenerator().randomExpandKNNQueriesClustered(graph, 4096, 1, 100.0 / graph.getNumNodes());
                queries.insert(queries.end(), queriesNew.begin(), queriesNew.end());
            }
            else if (query_c == 1)
            {
                queries = QueryGenerator().randomKNN(graph, 2048);
                auto queriesNew = QueryGenerator().randomExpandKNNQueriesClustered(graph, 6144, 1, 100.0 / graph.getNumNodes());
                queries.insert(queries.end(), queriesNew.begin(), queriesNew.end());
            }
            else if (query_c == 2)
            {
                queries = QueryGenerator().randomKNN(graph, 4096);
                auto queriesNew = QueryGenerator().randomExpandKNNQueriesClustered(graph, 4096, 2, 100.0 / graph.getNumNodes());
                queries.insert(queries.end(), queriesNew.begin(), queriesNew.end());
            }
            else if (query_c == 3)
            {
                queries = QueryGenerator().randomKNN(graph, 2048);
                auto queriesNew = QueryGenerator().randomExpandKNNQueriesClustered(graph, 6144, 2, 100.0 / graph.getNumNodes());
                queries.insert(queries.end(), queriesNew.begin(), queriesNew.end());
            }
            else if (query_c == 4)
            {
                queries = QueryGenerator().randomKNN(graph, 1024);
                auto queriesNew = QueryGenerator().randomExpandKNNQueriesClustered(graph, 7168, 2, 100.0 / graph.getNumNodes());
                queries.insert(queries.end(), queriesNew.begin(), queriesNew.end());
            }
            if (verifyKNN) {
                graph.resetAllObjects();
                graph.parseObjectSet(objects);
            }
            sw.reset();
            sw.start();
            experiment.buildIndex(graph);
            sw.stop();
            results[query_c].push_back(sw.getTimeMs());
            experiment.printInfo();
            experiment.clearObjects();
            experiment.loadObjects(graph, objects);
            for (auto queryNodeIt = queries.begin(); queryNodeIt != queries.end(); ++queryNodeIt) {
                kNNs.clear();
                kNNDistances.clear();
                kNNs.reserve(k);
                kNNDistances.reserve(k);
                sw.reset();
                sw.start();
                experiment.runQuery(graph, k, *queryNodeIt, kNNs, kNNDistances);
                sw.stop();
                totalQueryTime += sw.getTimeMs();

                if (verifyKNN) {
                    ineKNNs.clear();
                    ineKNNDistances.clear();
                    verifyExperiment.clearObjects();
                    verifyExperiment.loadObjects(graph, objects);
                    verifyExperiment.runQuery(graph, k, *queryNodeIt, ineKNNs, ineKNNDistances);
                    if (!utility::verifyKNN(ineKNNs, ineKNNDistances, kNNs, kNNDistances, false, k,
                                            message, true)) {
                        std::cout << "Verfication failed for query node " << *queryNodeIt << " with k = "
                                  << k
                                  << std::endl;
                        std::cout << "Message: " << message << std::endl;
                        exit(1);
                    }
                }
                queryCounter++;
                if (queryCounter == 4 || queryCounter == 16 || queryCounter == 64 ||
                    queryCounter == 256 || queryCounter == 1024 || queryCounter == 4196 || queryCounter == 8192) {
                    results[query_c].push_back(totalQueryTime);
                    std::cout << "Q: " << queryCounter << ", t: " << totalQueryTime << std::endl;

                }
            }
        }
    }
    write_to_csv2(results, std::vector<std::string>({"c=1, q_c=4096", "c=1, q_c=6144", "c=2, q_c=4096", "c=2, q_c=6144", "c=2, q_c=7168"}), output + "/agtree/" + graph.getNetworkName() + "_query-distribution_mixed", r);
}

void experimentKSize(Graph &graph, unsigned fanout = 4, unsigned maxLeafSize = 128, unsigned int r = 1, double density = 0.001, bool verifyKNN = false, std::string output = "")
{
    AdaptiveGTreeExperiment experiment(fanout, maxLeafSize);
    INEExperiment verifyExperiment;
    std::string message;


    std::vector <std::vector<double>> results(5);
    StopWatch sw;
    std::vector<NodeID> kNNs, ineKNNs;
    std::vector<EdgeWeight> kNNDistances, ineKNNDistances;
    unsigned queryCounter = 0;
    double totalQueryTime = 0.0;
    int i = 0;
    for (int k = 1; k < 17; k *= 2) {
        for (int repeats = 0; repeats < r; repeats++) {
            queryCounter = 0;
            totalQueryTime = 0.0;
            std::vector <NodeID> queries = QueryGenerator().randomKNN(graph, 8192);
            std::vector <NodeID> objects = QueryGenerator().randomObjects(graph, density);
            if (verifyKNN) {
                graph.resetAllObjects();
                graph.parseObjectSet(objects);
            }
            sw.reset();
            sw.start();
            experiment.buildIndex(graph);
            sw.stop();
            results[i].push_back(sw.getTimeMs());
            experiment.printInfo();
            experiment.clearObjects();
            experiment.loadObjects(graph, objects);
            for (auto queryNodeIt = queries.begin(); queryNodeIt != queries.end(); ++queryNodeIt) {
                kNNs.clear();
                kNNDistances.clear();
                kNNs.reserve(k);
                kNNDistances.reserve(k);
                sw.reset();
                sw.start();
                experiment.runQuery(graph, k, *queryNodeIt, kNNs, kNNDistances);
                sw.stop();
                totalQueryTime += sw.getTimeMs();

                if (verifyKNN) {
                    ineKNNs.clear();
                    ineKNNDistances.clear();
                    verifyExperiment.clearObjects();
                    verifyExperiment.loadObjects(graph, objects);
                    verifyExperiment.runQuery(graph, k, *queryNodeIt, ineKNNs, ineKNNDistances);
                    if (!utility::verifyKNN(ineKNNs, ineKNNDistances, kNNs, kNNDistances, false, k,
                                            message, true)) {
                        std::cout << "Verfication failed for query node " << *queryNodeIt << " with k = "
                                  << k
                                  << std::endl;
                        std::cout << "Message: " << message << std::endl;
                        exit(1);
                    }
                }
                queryCounter++;
                if (queryCounter == 4 || queryCounter == 16 || queryCounter == 64 ||
                    queryCounter == 256 || queryCounter == 1024 || queryCounter == 4196 || queryCounter == 8192) {
                    results[i].push_back(totalQueryTime);
                    std::cout << "Q: " << queryCounter << ", t: " << totalQueryTime << std::endl;
                }
            }
        }
        i++;
    }
    write_to_csv2(results, std::vector<std::string>({"k: 1", "k: 2", "k: 4", "k: 8", "k: 16"}), output + "/agtree/" + graph.getNetworkName() + "_k", r);
}

void experimentKSizeNA(Graph &graph, unsigned fanout = 4, unsigned maxLeafSize = 128, unsigned int r = 1, double density = 0.001, bool verifyKNN = false, std::string output = "")
{
    GTreeExperiment experiment(fanout, maxLeafSize);
    INEExperiment verifyExperiment;
    std::string message;


    std::vector <std::vector<double>> results(5);
    StopWatch sw;
    std::vector<NodeID> kNNs, ineKNNs;
    std::vector<EdgeWeight> kNNDistances, ineKNNDistances;
    unsigned queryCounter = 0;
    double totalQueryTime = 0.0;
    int i = 0;
    for (int k = 1; k < 17; k *= 2) {
        for (int repeats = 0; repeats < r; repeats++) {
            queryCounter = 0;
            totalQueryTime = 0.0;
            std::vector <NodeID> queries = QueryGenerator().randomKNN(graph, 8192);
            std::vector <NodeID> objects = QueryGenerator().randomObjects(graph, density);
            if (verifyKNN) {
                graph.resetAllObjects();
                graph.parseObjectSet(objects);
            }
            sw.reset();
            sw.start();
            experiment.buildIndex(graph);
            sw.stop();
            results[i].push_back(sw.getTimeMs());
            experiment.printInfo();
            experiment.clearObjects();
            experiment.loadObjects(graph, objects);
            for (auto queryNodeIt = queries.begin(); queryNodeIt != queries.end(); ++queryNodeIt) {
                kNNs.clear();
                kNNDistances.clear();
                kNNs.reserve(k);
                kNNDistances.reserve(k);
                sw.reset();
                sw.start();
                experiment.runQuery(graph, k, *queryNodeIt, kNNs, kNNDistances);
                sw.stop();
                totalQueryTime += sw.getTimeMs();

                if (verifyKNN) {
                    ineKNNs.clear();
                    ineKNNDistances.clear();
                    verifyExperiment.clearObjects();
                    verifyExperiment.loadObjects(graph, objects);
                    verifyExperiment.runQuery(graph, k, *queryNodeIt, ineKNNs, ineKNNDistances);
                    if (!utility::verifyKNN(ineKNNs, ineKNNDistances, kNNs, kNNDistances, false, k,
                                            message, true)) {
                        std::cout << "Verfication failed for query node " << *queryNodeIt << " with k = "
                                  << k
                                  << std::endl;
                        std::cout << "Message: " << message << std::endl;
                        exit(1);
                    }
                }
                queryCounter++;
                if (queryCounter == 4 || queryCounter == 16 || queryCounter == 64 ||
                    queryCounter == 256 || queryCounter == 1024 || queryCounter == 4196 || queryCounter == 8192) {
                    results[i].push_back(totalQueryTime);
                    std::cout << "Q: " << queryCounter << ", t: " << totalQueryTime << std::endl;
                }
            }
        }
        i++;
    }
    write_to_csv2(results, std::vector<std::string>({"k: 1", "k: 2", "k: 4", "k: 8", "k: 16"}), output + "/gtree/" + graph.getNetworkName() + "_k", r);
}

void experimentKSizeClusteredObjects(Graph &graph, unsigned fanout = 4, unsigned maxLeafSize = 128, unsigned int r = 1, double density = 0.001, bool verifyKNN = false, std::string output = "")
{
    AdaptiveGTreeExperiment experiment(fanout, maxLeafSize);
    INEExperiment verifyExperiment;
    std::string message;


    std::vector <std::vector<double>> results(5);
    StopWatch sw;
    std::vector<NodeID> kNNs, ineKNNs;
    std::vector<EdgeWeight> kNNDistances, ineKNNDistances;
    unsigned queryCounter = 0;
    double totalQueryTime = 0.0;
    int i = 0;
    for (int k = 1; k < 17; k *= 2) {
        for (int repeats = 0; repeats < r; repeats++) {
            queryCounter = 0;
            totalQueryTime = 0.0;
            std::vector <NodeID> queries = QueryGenerator().randomKNN(graph, 8192);
            std::vector <NodeID> objects = QueryGenerator().randomExpandObjectsClustered(graph, density, 4,
                                                                        100.0 / graph.getNumNodes());
            if (verifyKNN) {
                graph.resetAllObjects();
                graph.parseObjectSet(objects);
            }
            sw.reset();
            sw.start();
            experiment.buildIndex(graph);
            sw.stop();
            results[i].push_back(sw.getTimeMs());
            experiment.printInfo();
            experiment.clearObjects();
            experiment.loadObjects(graph, objects);
            for (auto queryNodeIt = queries.begin(); queryNodeIt != queries.end(); ++queryNodeIt) {
                kNNs.clear();
                kNNDistances.clear();
                kNNs.reserve(k);
                kNNDistances.reserve(k);
                sw.reset();
                sw.start();
                experiment.runQuery(graph, k, *queryNodeIt, kNNs, kNNDistances);
                sw.stop();
                totalQueryTime += sw.getTimeMs();

                if (verifyKNN) {
                    ineKNNs.clear();
                    ineKNNDistances.clear();
                    verifyExperiment.clearObjects();
                    verifyExperiment.loadObjects(graph, objects);
                    verifyExperiment.runQuery(graph, k, *queryNodeIt, ineKNNs, ineKNNDistances);
                    if (!utility::verifyKNN(ineKNNs, ineKNNDistances, kNNs, kNNDistances, false, k,
                                            message, true)) {
                        std::cout << "Verfication failed for query node " << *queryNodeIt << " with k = "
                                  << k
                                  << std::endl;
                        std::cout << "Message: " << message << std::endl;
                        exit(1);
                    }
                }
                queryCounter++;
                if (queryCounter == 4 || queryCounter == 16 || queryCounter == 64 ||
                    queryCounter == 256 || queryCounter == 1024 || queryCounter == 4196 || queryCounter == 8192) {
                    results[i].push_back(totalQueryTime);
                    std::cout << "Q: " << queryCounter << ", t: " << totalQueryTime << std::endl;
                }
            }
        }
        i++;
    }
    write_to_csv2(results, std::vector<std::string>({"k: 1", "k: 2", "k: 4", "k: 8", "k: 16"}), output + "/agtree/" + graph.getNetworkName() + "_k_objects-clustered", r);
}

void experimentKSizeClusteredObjectsNA(Graph &graph, unsigned fanout = 4, unsigned maxLeafSize = 128, unsigned int r = 1, double density = 0.001, bool verifyKNN = false, std::string output = "")
{
    GTreeExperiment experiment(fanout, maxLeafSize);
    INEExperiment verifyExperiment;
    std::string message;


    std::vector <std::vector<double>> results(5);
    StopWatch sw;
    std::vector<NodeID> kNNs, ineKNNs;
    std::vector<EdgeWeight> kNNDistances, ineKNNDistances;
    unsigned queryCounter = 0;
    double totalQueryTime = 0.0;
    int i = 0;
    for (int k = 1; k < 17; k *= 2) {
        for (int repeats = 0; repeats < r; repeats++) {
            queryCounter = 0;
            totalQueryTime = 0.0;
            std::vector <NodeID> queries = QueryGenerator().randomKNN(graph, 8192);
            std::vector <NodeID> objects = QueryGenerator().randomExpandObjectsClustered(graph, density, 4,
                                                                100.0 / graph.getNumNodes());
            if (verifyKNN) {
                graph.resetAllObjects();
                graph.parseObjectSet(objects);
            }
            sw.reset();
            sw.start();
            experiment.buildIndex(graph);
            sw.stop();
            results[i].push_back(sw.getTimeMs());
            experiment.printInfo();
            experiment.clearObjects();
            experiment.loadObjects(graph, objects);
            for (auto queryNodeIt = queries.begin(); queryNodeIt != queries.end(); ++queryNodeIt) {
                kNNs.clear();
                kNNDistances.clear();
                kNNs.reserve(k);
                kNNDistances.reserve(k);
                sw.reset();
                sw.start();
                experiment.runQuery(graph, k, *queryNodeIt, kNNs, kNNDistances);
                sw.stop();
                totalQueryTime += sw.getTimeMs();

                if (verifyKNN) {
                    ineKNNs.clear();
                    ineKNNDistances.clear();
                    verifyExperiment.clearObjects();
                    verifyExperiment.loadObjects(graph, objects);
                    verifyExperiment.runQuery(graph, k, *queryNodeIt, ineKNNs, ineKNNDistances);
                    if (!utility::verifyKNN(ineKNNs, ineKNNDistances, kNNs, kNNDistances, false, k,
                                            message, true)) {
                        std::cout << "Verfication failed for query node " << *queryNodeIt << " with k = "
                                  << k
                                  << std::endl;
                        std::cout << "Message: " << message << std::endl;
                        exit(1);
                    }
                }
                queryCounter++;
                if (queryCounter == 4 || queryCounter == 16 || queryCounter == 64 ||
                    queryCounter == 256 || queryCounter == 1024 || queryCounter == 4196 || queryCounter == 8192) {
                    results[i].push_back(totalQueryTime);
                    std::cout << "Q: " << queryCounter << ", t: " << totalQueryTime << std::endl;
                }
            }
        }
        i++;
    }
    write_to_csv2(results, std::vector<std::string>({"k: 1", "k: 2", "k: 4", "k: 8", "k: 16"}), output + "/gtree/" + graph.getNetworkName() + "_k_objects-clustered", r);
}

void experimentObjectSize(Graph &graph, unsigned fanout = 4, unsigned maxLeafSize = 128, unsigned int r = 1, unsigned k = 10, bool verifyKNN = false, std::string output = "")
{
    AdaptiveGTreeExperiment experiment(fanout, maxLeafSize);
    INEExperiment verifyExperiment;
    std::string message;


    std::vector <std::vector<double>> results(5);
    StopWatch sw;
    std::vector<NodeID> kNNs, ineKNNs;
    std::vector<EdgeWeight> kNNDistances, ineKNNDistances;
    unsigned queryCounter = 0;
    double totalQueryTime = 0.0;
    int i = 0;
    for (double density = 0.0001; density < 1.01; density *= 10) {
        for (int repeats = 0; repeats < r; repeats++) {
            queryCounter = 0;
            totalQueryTime = 0.0;
            std::vector <NodeID> queries = QueryGenerator().randomKNN(graph, 8192);
            std::vector <NodeID> objects = QueryGenerator().randomObjects(graph, density);
            if (verifyKNN) {
                graph.resetAllObjects();
                graph.parseObjectSet(objects);
            }
            sw.reset();
            sw.start();
            experiment.buildIndex(graph);
            sw.stop();
            results[i].push_back(sw.getTimeMs());
            experiment.printInfo();
            experiment.clearObjects();
            experiment.loadObjects(graph, objects);
            for (auto queryNodeIt = queries.begin(); queryNodeIt != queries.end(); ++queryNodeIt) {
                kNNs.clear();
                kNNDistances.clear();
                kNNs.reserve(k);
                kNNDistances.reserve(k);
                sw.reset();
                sw.start();
                experiment.runQuery(graph, k, *queryNodeIt, kNNs, kNNDistances);
                sw.stop();
                totalQueryTime += sw.getTimeMs();

                if (verifyKNN) {
                    ineKNNs.clear();
                    ineKNNDistances.clear();
                    verifyExperiment.clearObjects();
                    verifyExperiment.loadObjects(graph, objects);
                    verifyExperiment.runQuery(graph, k, *queryNodeIt, ineKNNs, ineKNNDistances);
                    if (!utility::verifyKNN(ineKNNs, ineKNNDistances, kNNs, kNNDistances, false, k,
                                            message, true)) {
                        std::cout << "Verfication failed for query node " << *queryNodeIt << " with k = "
                                  << k
                                  << std::endl;
                        std::cout << "Message: " << message << std::endl;
                        exit(1);
                    }
                }
                queryCounter++;
                if (queryCounter == 4 || queryCounter == 16 || queryCounter == 64 ||
                    queryCounter == 256 || queryCounter == 1024 || queryCounter == 4196 || queryCounter == 8192) {
                    results[i].push_back(totalQueryTime);
                    std::cout << "Q: " << queryCounter << ", t: " << totalQueryTime << std::endl;
                }
            }
        }
        i++;
    }
    write_to_csv2(results, std::vector<std::string>({"density: 0.0001", "density: 0.001", "density: 0.01", "density: 0.1", "density: 1.0"}), output + "/agtree/" + graph.getNetworkName() + "_density", r);
}

void experimentObjectSizeNA(Graph &graph, unsigned fanout = 4, unsigned maxLeafSize = 128, unsigned int r = 1, unsigned k = 10, bool verifyKNN = false, std::string output = "")
{
    GTreeExperiment experiment(fanout, maxLeafSize);
    INEExperiment verifyExperiment;
    std::string message;


    std::vector <std::vector<double>> results(5);
    StopWatch sw;
    std::vector<NodeID> kNNs, ineKNNs;
    std::vector<EdgeWeight> kNNDistances, ineKNNDistances;
    unsigned queryCounter = 0;
    double totalQueryTime = 0.0;
    int i = 0;
    for (double density = 0.0001; density < 1.01; density *= 10) {
        for (int repeats = 0; repeats < r; repeats++) {
            queryCounter = 0;
            totalQueryTime = 0.0;
            std::vector <NodeID> queries = QueryGenerator().randomKNN(graph, 8192);
            std::vector <NodeID> objects = QueryGenerator().randomObjects(graph, density);
            if (verifyKNN) {
                graph.resetAllObjects();
                graph.parseObjectSet(objects);
            }
            sw.reset();
            sw.start();
            experiment.buildIndex(graph);
            sw.stop();
            results[i].push_back(sw.getTimeMs());
            experiment.printInfo();
            experiment.clearObjects();
            experiment.loadObjects(graph, objects);
            for (auto queryNodeIt = queries.begin(); queryNodeIt != queries.end(); ++queryNodeIt) {
                kNNs.clear();
                kNNDistances.clear();
                kNNs.reserve(k);
                kNNDistances.reserve(k);
                sw.reset();
                sw.start();
                experiment.runQuery(graph, k, *queryNodeIt, kNNs, kNNDistances);
                sw.stop();
                totalQueryTime += sw.getTimeMs();

                if (verifyKNN) {
                    ineKNNs.clear();
                    ineKNNDistances.clear();
                    verifyExperiment.clearObjects();
                    verifyExperiment.loadObjects(graph, objects);
                    verifyExperiment.runQuery(graph, k, *queryNodeIt, ineKNNs, ineKNNDistances);
                    if (!utility::verifyKNN(ineKNNs, ineKNNDistances, kNNs, kNNDistances, false, k,
                                            message, true)) {
                        std::cout << "Verfication failed for query node " << *queryNodeIt << " with k = "
                                  << k
                                  << std::endl;
                        std::cout << "Message: " << message << std::endl;
                        exit(1);
                    }
                }
                queryCounter++;
                if (queryCounter == 4 || queryCounter == 16 || queryCounter == 64 ||
                    queryCounter == 256 || queryCounter == 1024 || queryCounter == 4196 || queryCounter == 8192) {
                    results[i].push_back(totalQueryTime);
                    std::cout << "Q: " << queryCounter << ", t: " << totalQueryTime << std::endl;
                }
            }
        }
        i++;
    }
    write_to_csv2(results, std::vector<std::string>({"density: 0.0001", "density: 0.001", "density: 0.01", "density: 0.1", "density: 1.0"}), output + "/gtree/" + graph.getNetworkName() + "_density", r);
}

void experimentFAndTau(Graph& graph, unsigned int r = 1, double density = 0.001, unsigned k = 10, bool verifyKNN = false, int fanout = 2, std::string output = "")
{
    INEExperiment verifyExperiment;
    std::string message;

    std::cout << "v: " << std::to_string(verifyKNN) << ", k: " << std::to_string(k) << ", d: "
              << std::to_string(density) << std::endl;


    std::vector <std::vector<double>> results(4);
    StopWatch sw;
    std::vector <NodeID> kNNs, ineKNNs;
    std::vector <EdgeWeight> kNNDistances, ineKNNDistances;
    unsigned queryCounter = 0;
    double totalQueryTime = 0.0;
    int i = 0;
    for (int tau = 32; tau < 257; tau *= 2) {
        AdaptiveGTreeExperiment experiment(fanout, tau);
        for (int repeats = 0; repeats < r; repeats++) {
            std::cout << "Fanout: " << fanout << ", tau: " << tau << ", repeat: " << repeats + 1 << std::endl;
            queryCounter = 0;
            totalQueryTime = 0.0;
            std::vector <NodeID> queries = QueryGenerator().randomKNN(graph, 8192);
            std::vector <NodeID> objects = QueryGenerator().randomObjects(graph, density);
            if (verifyKNN) {
                graph.resetAllObjects();
                graph.parseObjectSet(objects);
            }
            sw.reset();
            sw.start();
            experiment.buildIndex(graph);
            sw.stop();
            results[i].push_back(sw.getTimeMs());
            experiment.printInfo();
            experiment.clearObjects();
            experiment.loadObjects(graph, objects);
            for (auto queryNodeIt = queries.begin(); queryNodeIt != queries.end(); ++queryNodeIt) {
                kNNs.clear();
                kNNDistances.clear();
                kNNs.reserve(k);
                kNNDistances.reserve(k);
                sw.reset();
                sw.start();
                experiment.runQuery(graph, k, *queryNodeIt, kNNs, kNNDistances);
                sw.stop();
                totalQueryTime += sw.getTimeMs();

                if (verifyKNN) {
                    ineKNNs.clear();
                    ineKNNDistances.clear();
                    verifyExperiment.clearObjects();
                    verifyExperiment.loadObjects(graph, objects);
                    verifyExperiment.runQuery(graph, k, *queryNodeIt, ineKNNs, ineKNNDistances);
                    if (!utility::verifyKNN(ineKNNs, ineKNNDistances, kNNs, kNNDistances, false, k,
                                            message, true)) {
                        std::cout << "Verfication failed for query node " << *queryNodeIt << " with k = "
                                  << k
                                  << std::endl;
                        std::cout << "Message: " << message << std::endl;
                        exit(1);
                    }
                }
                queryCounter++;
                if (queryCounter == 4 || queryCounter == 16 || queryCounter == 64 ||
                    queryCounter == 256 || queryCounter == 1024 || queryCounter == 4196 || queryCounter == 8192) {
                    results[i].push_back(totalQueryTime);
                    std::cout << "Q: " << queryCounter << ", t: " << totalQueryTime << std::endl;
                }
            }
        }
        i++;
    }
    write_to_csv2(results, std::vector<std::string>({"tau: 32", "tau: 64", "tau: 128", "tau: 256"}),
                  output + "/agtree/" + graph.getNetworkName() + "_fanout_" + std::to_string(fanout), r);
    results.clear();
}

void experimentDistanceMatrixConvergence(Graph& graph, std::string output = "", unsigned r = 1, unsigned fanout=4, unsigned maxLeafSize=128, unsigned k=10, double density=0.001, bool verifyKNN = false)
{
    AdaptiveGTreeExperiment experiment(fanout, maxLeafSize);
    INEExperiment verifyExperiment;
    std::string message;
    std::vector < std::vector< std::tuple< unsigned, unsigned, unsigned, unsigned > > > results;
    std::vector<NodeID> kNNs, ineKNNs;
    std::vector<EdgeWeight> kNNDistances, ineKNNDistances;
    unsigned queryCounter = 0;
    unsigned totalQueryTime = 0.0;

    for (int repeats = 0; repeats < r; repeats++) {
        queryCounter = 0;
        totalQueryTime = 0.0;
        std::vector <NodeID> queries = QueryGenerator().randomKNN(graph, 8192);
        std::vector <NodeID> objects = QueryGenerator().randomObjects(graph, density);
        experiment.buildIndex(graph);
        experiment.printInfo();
        experiment.clearObjects();
        experiment.loadObjects(graph, objects);
        for (auto queryNodeIt = queries.begin(); queryNodeIt != queries.end(); ++queryNodeIt) {
            kNNs.clear();
            kNNDistances.clear();
            kNNs.reserve(k);
            kNNDistances.reserve(k);
            if (queryCounter == 0) {
                std::cout << "Q: " << queryCounter << std::endl;
                experiment.getConvergence(results);
            }
            experiment.runQuery(graph, k, *queryNodeIt, kNNs, kNNDistances);

            queryCounter++;
            if (queryCounter < 11) {
                std::cout << "Q: " << queryCounter << std::endl;
                experiment.getConvergence(results);
            }
            else if (queryCounter < 101 && queryCounter % 10 == 0) {
                std::cout << "Q: " << queryCounter << std::endl;
                experiment.getConvergence(results);
            }
            else if (queryCounter < 1001 && queryCounter % 100 == 0) {
                std::cout << "Q: " << queryCounter << std::endl;
                experiment.getConvergence(results);
            }
            else if (queryCounter < 10001 && queryCounter % 1000 == 0) {
                std::cout << "Q: " << queryCounter << std::endl;
                experiment.getConvergence(results);
            }
        }
    }
    int measurements = results[0].size() / r;
    std::vector<std::vector<double>> leafResults(results.size());
    std::vector<std::vector<double>> internalResults(results.size());
    std::vector<std::string> columnNames(results.size());
    for (int i = 0; i < results.size(); i++) {
        for (int j = 0; j < measurements; j++) {
            double avgFilledLeaf = 0.0;
            double avgFilledInternal = 0.0;
            for (int k = 0; k < r; k++) {
                auto cell = results[i][j + k * measurements];
                avgFilledLeaf += std::get<0>(cell) != 0 ? (std::get<1>(cell) / ((double) std::get<0>(cell))) : 0.0;
                avgFilledInternal += std::get<2>(cell) != 0 ? std::get<3>(cell) / ((double) std::get<2>(cell)) : 0.0;
            }
            std::cout << std::endl;
            leafResults[i].push_back(avgFilledLeaf / r);
            internalResults[i].push_back(avgFilledInternal / r);
        }
    }
    for (int i = 0; i < results.size(); i++){
        columnNames[i] = "level " + std::to_string(i + 1);;
    }
    columnNames[results.size() - 1] = "total";

    write_to_csv2(leafResults, columnNames, output + "/agtree/" + graph.getNetworkName() + "_convergence-leaf", 1);
    write_to_csv2(internalResults, columnNames, output + "/agtree/" + graph.getNetworkName() + "_convergence-internal", 1);
}

void experimentDistanceMatrixConvergenceClusteredQueries(Graph& graph, std::string output = "", unsigned r = 1, unsigned fanout=4, unsigned maxLeafSize=128, unsigned k=10, double density=0.001, bool verifyKNN = false)
{
    AdaptiveGTreeExperiment experiment(fanout, maxLeafSize);
    INEExperiment verifyExperiment;
    std::string message;
    std::vector < std::vector< std::tuple< unsigned, unsigned, unsigned, unsigned > > > results;
    std::vector<NodeID> kNNs, ineKNNs;
    std::vector<EdgeWeight> kNNDistances, ineKNNDistances;
    unsigned queryCounter = 0;
    unsigned totalQueryTime = 0.0;

    for (int repeats = 0; repeats < r; repeats++) {
        queryCounter = 0;
        totalQueryTime = 0.0;
        std::vector <NodeID> queries = QueryGenerator().randomExpandKNNQueriesClustered(graph, 8192, 4, 100.0 / graph.getNumNodes());
        std::vector <NodeID> objects = QueryGenerator().randomObjects(graph, density);
        experiment.buildIndex(graph);
        experiment.printInfo();
        experiment.clearObjects();
        experiment.loadObjects(graph, objects);
        for (auto queryNodeIt = queries.begin(); queryNodeIt != queries.end(); ++queryNodeIt) {
            kNNs.clear();
            kNNDistances.clear();
            kNNs.reserve(k);
            kNNDistances.reserve(k);
            if (queryCounter == 0) {
                std::cout << "Q: " << queryCounter << std::endl;
                experiment.getConvergence(results);
            }
            experiment.runQuery(graph, k, *queryNodeIt, kNNs, kNNDistances);

            queryCounter++;
            if (queryCounter < 11) {
                std::cout << "Q: " << queryCounter << std::endl;
                experiment.getConvergence(results);
            }
            else if (queryCounter < 101 && queryCounter % 10 == 0) {
                std::cout << "Q: " << queryCounter << std::endl;
                experiment.getConvergence(results);
            }
            else if (queryCounter < 1001 && queryCounter % 100 == 0) {
                std::cout << "Q: " << queryCounter << std::endl;
                experiment.getConvergence(results);
            }
            else if (queryCounter < 10001 && queryCounter % 1000 == 0) {
                std::cout << "Q: " << queryCounter << std::endl;
                experiment.getConvergence(results);
            }
        }
    }
    int measurements = results[0].size() / r;
    std::vector<std::vector<double>> leafResults(results.size());
    std::vector<std::vector<double>> internalResults(results.size());
    std::vector<std::string> columnNames(results.size());
    for (int i = 0; i < results.size(); i++) {
        for (int j = 0; j < measurements; j++) {
            double avgFilledLeaf = 0.0;
            double avgFilledInternal = 0.0;
            for (int k = 0; k < r; k++) {
                auto cell = results[i][j + k * measurements];
                avgFilledLeaf += std::get<0>(cell) != 0 ? (std::get<1>(cell) / ((double) std::get<0>(cell))) : 0.0;
                avgFilledInternal += std::get<2>(cell) != 0 ? std::get<3>(cell) / ((double) std::get<2>(cell)) : 0.0;
            }
            std::cout << std::endl;
            leafResults[i].push_back(avgFilledLeaf / r);
            internalResults[i].push_back(avgFilledInternal / r);
        }
    }
    for (int i = 0; i < results.size(); i++){
        columnNames[i] = "level " + std::to_string(i + 1);;
    }
    columnNames[results.size() - 1] = "total";


    write_to_csv2(leafResults, columnNames, output + "/agtree/" + graph.getNetworkName() + "_convergence-leaf_queries-clustered", 1);
    write_to_csv2(internalResults, columnNames, output + "/agtree/" + graph.getNetworkName() + "_convergence-internal_queries-clustered", 1);
}




void AdaptiveExperimentsCommand::execute(int argc, char *argv[])
{
    std::string experiment = "";
    std::string bgrFilePath = "";
    std::string filePathPrefix = "";
    std::string statsOutputFile = "";
    std::string rwPOISetListFile = "";
    std::string parameters = "";
    std::string objDensities = "";
    std::string objTypes = "";
    std::string objVariable = "";
    unsigned int numSets = 0;
    std::string queryNodeFile = "";
    std::string kValues = "";
    std::string method = "";
    unsigned int numPoints = 0;
    unsigned numRepetitions = 5;
    unsigned fanout = 4;
    unsigned tau = 128;
    unsigned _method = 0;
    double density = 0.001;
    unsigned k = 10;
    int verify = 0;
    std::string special = "";

    bool experimentalExperiments = false;

    /*
     * Process Command Line Arguments
     */
    int opt;
    while ((opt = getopt(argc, argv, "e:g:p:f:s:n:d:t:q:k:m:v:l:r:E:M:R:F:T:D:K:V:S:")) != -1) {
        switch (opt) {
            case 'e':
                experiment = optarg;
                break;
            case 'g':
                bgrFilePath = optarg;
                break;
            case 'p':
                parameters = optarg;
                break;
            case 'f':
                filePathPrefix = optarg;
                break;
            case 's':
                statsOutputFile = optarg;
                break;
            case 'n':
                numSets = std::stoul(optarg);
                break;
            case 'd':
                objDensities = optarg;
                break;
            case 't':
                objTypes = optarg;
                break;
            case 'q':
                queryNodeFile = optarg;
                break;
            case 'k':
                kValues = optarg;
                break;
            case 'm':
                method = optarg;
                break;
            case 'v':
                objVariable = optarg;
                break;
            case 'l':
                numPoints = std::stoul(optarg);
                break;
            case 'r':
                rwPOISetListFile = optarg;
                break;
            case 'E':
                experimentalExperiments = true;
                break;
            case 'M':
                _method = std::stoi(optarg);
            case 'R':
                numRepetitions = std::stoi(optarg);
                break;
            case 'F':
                fanout = std::stoi(optarg);
                break;
            case 'T':
                tau = std::stoi(optarg);
                break;
            case 'D':
                density = std::stod(optarg);
                break;
            case 'K':
                k = std::stoi(optarg);
                break;
            case 'V':
                verify = std::stoi(optarg);
                break;
            case 'S':
                special = optarg;
                break;
            default:
                std::cerr << "Unknown option(s) provided!\n\n";
                showCommandUsage(argv[0]);
                exit(1);
        }
    }

    if (experimentalExperiments) {
        this->runSingleMethodQueries(bgrFilePath, _method, density, numRepetitions, tau, fanout, k, verify, special, filePathPrefix);
        exit(1);
    }

    // Validate Command Line Arguments
    if (argc == 5) {
        // This is 5 so that user can just enter method to find out what parameters are required for method
        this->showPhaseUsage(experiment, argv[0]);
        exit(1);
    }

    if (experiment == "") {
        std::cerr << "Invalid argument(s)!\n\n";
        this->showCommandUsage(argv[0]);
        exit(1);
    }

    if (experiment == constants::EXP_RUN_KNN_OPTIMIZATIONS) {
        if (argc < 27) {
            // Arguments: -m <method> -g <binary graph file> -q <query node file>
            // -k <k values> -p <parameters> -n <num sets> -d <list of object densities> -t <list of object types> -v <list of some object variable>
            // -f <index output file path prefix> -s <stats output file>
            std::cerr << "Too few arguments!\n\n";
            this->showPhaseUsage(experiment, argv[0]);
            exit(1);
        }

        if (bgrFilePath == "" || parameters == "" || filePathPrefix == "" || statsOutputFile == ""
            || numSets == 0 || objDensities == "" || objTypes == "" || queryNodeFile == ""
            || kValues == "" || method == "") {
            std::cerr << "Invalid argument(s)!\n\n";
            this->showPhaseUsage(experiment, argv[0]);
            exit(1);
        }

        this->runSingleMethodQueries(method, bgrFilePath, queryNodeFile, kValues, parameters, numSets, objDensities,
                                     objTypes, objVariable, filePathPrefix, statsOutputFile);

    } else {
        std::cerr << "Invalid experimental step!\n\n";
        this->showCommandUsage(argv[0]);
        exit(1);
    }

}

void AdaptiveExperimentsCommand::showCommandUsage(std::string programName)
{
    std::cerr << "Usage: " << programName << " -c " + constants::EXPERIMENTS_CMD + " -e <experimental step>\n\n"
              << "Steps:\n"
              << utility::getFormattedUsageString(constants::EXP_BUILD_INDEXES, "1. Build all indexes") + "\n"
              << utility::getFormattedUsageString(constants::EXP_BUILD_OBJ_INDEXES,
                                                  "2. Generate object sets and build object indexes") + "\n"
              << utility::getFormattedUsageString(constants::EXP_RUN_KNN, "3. Run all standard kNN query experiments") +
                 "\n"
              << utility::getFormattedUsageString(constants::EXP_RUN_KNN_OPTIMIZATIONS,
                                                  "4. Run individual method kNN query experiments") + "\n"
              << utility::getFormattedUsageString(constants::EXP_RUN_KNN_RW_POI,
                                                  "5. Build object indexes for real-world POIs") + "\n"
              << utility::getFormattedUsageString(constants::EXP_RUN_KNN_RW_POI,
                                                  "6. Run kNN querying on real-world POIs experiments") + "\n";
}

void AdaptiveExperimentsCommand::showPhaseUsage(std::string method, std::string programName)
{
    if (method == constants::EXP_RUN_KNN) {
        std::cerr << "Usage: " << programName << " -c " + constants::EXPERIMENTS_CMD
                  << " -e " + constants::EXP_RUN_KNN + " -g <binary graph file>\n"
                  << "-q <query node file> -k <k values> -p <parameters> -n <num sets> -d <list of object set densities or partitions>\n"
                  << "-t <list of object types>  -v <list of some object variable> -f <index output file path prefix> -s <stats output file>\n";
    } else if (method == constants::EXP_RUN_KNN_OPTIMIZATIONS) {
        std::cerr << "Usage: " << programName << " -c " + constants::EXPERIMENTS_CMD
                  << " -e " + constants::EXP_RUN_KNN_OPTIMIZATIONS + " -m <method>\n"
                  << "-g <binary graph file> -q <query node file> -k <k values> -p <parameters>\n"
                  << "-n <num sets> -d <list of object densities> -t <list of object types>\n"
                  << " -v <list of some object variable> -f <index output file path prefix> -s <stats output file>\n";
    } else {
        std::cerr << "Invalid experiment phase!" << std::endl;
        this->showCommandUsage(programName);
    }
}

std::unordered_map<std::string, std::string> AdaptiveExperimentsCommand::getParameters(std::string parameters)
{
    std::unordered_map<std::string, std::string> parameterMap;
    std::vector<std::string> pairs = utility::splitByDelim(parameters, ',');

    for (std::size_t i = 0; i < pairs.size(); ++i) {
        std::vector<std::string> pair = utility::splitByDelim(pairs[i], '=');
        if (pair.size() == 2) {
//             std::cout << "Key = " << pair[0] << std::endl;
//             std::cout << "Value = " << pair[1] << std::endl;
            parameterMap[pair[0]] = pair[1];
        } else {
            std::cerr << "Invalid key-value pair in parameter string: " << pair[i] << std::endl;
            exit(1);
        }
    }
    return parameterMap;
}


void AdaptiveExperimentsCommand::runSingleMethodQueries(std::string bgrFileName, unsigned method, double density = 0.001, unsigned r = 5, unsigned tau = 128, unsigned fanout = 4, unsigned k = 10, int verify = 0, std::string special = "", std::string output = "")
{
    graph = serialization::getIndexFromBinaryFile<Graph>(bgrFileName);

    std::cout << "--- Running experiment, method: " << std::to_string(method) << ", network: " << graph.getNetworkName() << " ---"
              << std::endl;

    bool verifyKNN = verify == 1;

    switch (method) {
        case 0:
            if (special != "")
                experimentFAndTau(graph, r, density, k, verifyKNN, std::stoi(special), output);
            break;
        case 1:
            experimentObjectSize(graph, fanout, tau, r, k, verifyKNN, output);
            experimentObjectSizeNA(graph, fanout, tau, r, k, verifyKNN, output);
            break;
        case 2:
            experimentKSize(graph, fanout, tau, r, density, verifyKNN, output);
            experimentKSizeNA(graph, fanout, tau, r, density, verifyKNN, output);
            break;
        case 3:
            experimentObjectDistribution(graph, fanout, tau, r, density, k, verifyKNN, output);
            experimentObjectDistributionNA(graph, fanout, tau, r, density, k, verifyKNN, output);
            break;
        case 4:
        //those
            experimentQueryDistribution(graph, fanout, tau, r, density, k, verifyKNN, output);
            experimentQueryDistributionNA(graph, fanout, tau, r, density, k, verifyKNN, output);
            break;
        case 5:
            experimentDistanceMatrixConvergence(graph, output, r);
            experimentDistanceMatrixConvergenceClusteredQueries(graph, output, r);
            break;
        case 6:
            experimentKSizeClusteredObjects(graph, fanout, tau, r, density, verifyKNN, output);
            experimentKSizeClusteredObjectsNA(graph, fanout, tau, r, density, verifyKNN, output);
            break;
        case 7:
            if (special != "") {
                int exp = std::stoi(special);
                std::cout << exp << std::endl;
                if (exp == 0) {
                    AdaptiveGTreeExperiment experiment(4, 128);
                    experimentCompareKnn(experiment, graph, output, r, exp);
                } else if (exp == 1) {
                    GTreeExperiment experiment(4, 128);
                    experimentCompareKnn(experiment, graph, output, r, exp);
                } else if (exp == 2) {
                    INEExperiment experiment;
                    experimentCompareKnn(experiment, graph, output, r, exp);
                } else if (exp == 3) {
                    IERAStarExperiment experiment(4);
                    experimentCompareKnn(experiment, graph, output, r, exp);
                } else if (exp == 4) {
                    IERALTExperiment experiment(4, 8, LANDMARK_TYPE::FARTHEST, ALTParameters::buildKnnParameters());
                    experimentCompareKnn(experiment, graph, output, r, exp);
                } else if (exp == 5) {
                    IERALTExperiment experiment(4, 8, LANDMARK_TYPE::AVOID, ALTParameters::buildKnnParameters());
                    experimentCompareKnn(experiment, graph, output, r, exp);
                } else if (exp == 6) {
                    IERALTExperiment experiment(4, 20, LANDMARK_TYPE::FARTHEST, ALTParameters::buildKnnParameters());
                    experimentCompareKnn(experiment, graph, output, r, exp);
                } else if (exp == 7) {
                    IERALTExperiment experiment(4, 20, LANDMARK_TYPE::AVOID, ALTParameters::buildKnnParameters());
                    experimentCompareKnn(experiment, graph, output, r, exp);
                }
            }
            break;
        case 8:
            if (special != "") {
                int exp = std::stoi(special);
                if (exp == 0) {
                    AdaptiveGTreeExperiment experiment(4, 128);
                    experimentCompareKnnClusteredObjects(experiment, graph, output, r, exp);
                } else if (exp == 1) {
                    GTreeExperiment experiment(4, 128);
                    experimentCompareKnnClusteredObjects(experiment, graph, output, r, exp);
                } else if (exp == 2) {
                    INEExperiment experiment;
                    experimentCompareKnnClusteredObjects(experiment, graph, output, r, exp);
                } else if (exp == 3) {
                    IERAStarExperiment experiment(4);
                    experimentCompareKnnClusteredObjects(experiment, graph, output, r, exp, 64);
                } else
                    if (exp == 4) {
                    IERALTExperiment experiment(4, 8, LANDMARK_TYPE::FARTHEST, ALTParameters::buildKnnParameters());
                    experimentCompareKnnClusteredObjects(experiment, graph, output, r, exp, 1024);
                }
                else if (exp == 5) {
                    IERALTExperiment experiment(4, 8, LANDMARK_TYPE::AVOID, ALTParameters::buildKnnParameters());
                    experimentCompareKnnClusteredObjects(experiment, graph, output, r, exp, 1024);
                } else if (exp == 6) {
                    IERALTExperiment experiment(4, 20, LANDMARK_TYPE::FARTHEST, ALTParameters::buildKnnParameters());
                    experimentCompareKnnClusteredObjects(experiment, graph, output, r, exp), 1024;
                } else if (exp == 7) {
                    IERALTExperiment experiment(4, 20, LANDMARK_TYPE::AVOID, ALTParameters::buildKnnParameters());
                    experimentCompareKnnClusteredObjects(experiment, graph, output, r, exp, 1024);
                }
            }
            break;
        case 9:
            if (special != "") {
                int exp = std::stoi(special);
                if (exp == 0) {
                    AdaptiveGTreeExperiment experiment(4, 128);
                    experimentCompareKnnClusteredQueries(experiment, graph, output, r, exp);
                } else if (exp == 1) {
                    GTreeExperiment experiment(4, 128);
                    experimentCompareKnnClusteredQueries(experiment, graph, output, r, exp);
                } else if (exp == 2) {
                    INEExperiment experiment;
                    experimentCompareKnnClusteredQueries(experiment, graph, output, r, exp);
                } else if (exp == 3) {
                    IERAStarExperiment experiment(4);
                    experimentCompareKnnClusteredQueries(experiment, graph, output, r, exp);
                } else if (exp == 4) {
                    IERALTExperiment experiment(4, 8, LANDMARK_TYPE::FARTHEST, ALTParameters::buildKnnParameters());
                    experimentCompareKnnClusteredQueries(experiment, graph, output, r, exp);
                } else if (exp == 5) {
                    IERALTExperiment experiment(4, 8, LANDMARK_TYPE::AVOID, ALTParameters::buildKnnParameters());
                    experimentCompareKnnClusteredQueries(experiment, graph, output, r, exp);
                } else if (exp == 6) {
                    IERALTExperiment experiment(4, 20, LANDMARK_TYPE::FARTHEST, ALTParameters::buildKnnParameters());
                    experimentCompareKnnClusteredQueries(experiment, graph, output, r, exp);
                } else if (exp == 7) {
                    IERALTExperiment experiment(4, 20, LANDMARK_TYPE::AVOID, ALTParameters::buildKnnParameters());
                    experimentCompareKnnClusteredQueries(experiment, graph, output, r, exp);
                }
            }
            break;
        case 10:
            if (special != "") {
                int exp = std::stoi(special);
                if (exp == 0) {
                    AdaptiveGTreeExperiment experiment(4, 128);
                    experimentCompareKnnClustered(experiment, graph, output, r, exp);
                } else if (exp == 1) {
                    GTreeExperiment experiment(4, 128);
                    experimentCompareKnnClustered(experiment, graph, output, r, exp);
                } else if (exp == 2) {
                    INEExperiment experiment;
                    experimentCompareKnnClustered(experiment, graph, output, r, exp);
                } else if (exp == 3) {
                    IERAStarExperiment experiment(4);
                    experimentCompareKnnClustered(experiment, graph, output, r, exp);
                } else if (exp == 4) {
                    IERALTExperiment experiment(4, 8, LANDMARK_TYPE::FARTHEST, ALTParameters::buildKnnParameters());
                    experimentCompareKnnClustered(experiment, graph, output, r, exp);
                } else if (exp == 5) {
                    IERALTExperiment experiment(4, 8, LANDMARK_TYPE::AVOID, ALTParameters::buildKnnParameters());
                    experimentCompareKnnClustered(experiment, graph, output, r, exp);
                } else if (exp == 6) {
                    IERALTExperiment experiment(4, 20, LANDMARK_TYPE::FARTHEST, ALTParameters::buildKnnParameters());
                    experimentCompareKnnClustered(experiment, graph, output, r, exp);
                } else if (exp == 7) {
                    IERALTExperiment experiment(4, 20, LANDMARK_TYPE::AVOID, ALTParameters::buildKnnParameters());
                    experimentCompareKnnClustered(experiment, graph, output, r, exp);
                }
            }
            break;
        case 11:
            experimentObjectDistributionMixed(graph, fanout, tau, r, density, k, verifyKNN, output);
            experimentQueryDistributionMixed(graph, fanout, tau, r, density, k, verifyKNN, output);
            break;
        default:
            break;
    }
}


void AdaptiveExperimentsCommand::runSingleMethodQueries(std::string method, std::string bgrFileName,
                                                        std::string queryNodeFile, std::string kValues,
                                                        std::string parameters,
                                                        std::size_t numSets, std::string objDensities,
                                                        std::string objTypes, std::string objVariable,
                                                        std::string filePathPrefix, std::string statsOutputFile)
{
    graph = serialization::getIndexFromBinaryFile<Graph>(bgrFileName);

    std::cout << "--- Running experiment, method: " << method << ", network: " << graph.getNetworkName() << " ---"
              << std::endl;

    std::unordered_map<std::string, std::string> parameterMap = this->getParameters(parameters);

    queryNodes = utility::getPointSetFromFile(queryNodeFile);

    this->verifyKNN = parameterMap["verify"] == "1";

    // Find all additional fields we need to add to kNN stats tuples
    std::vector<std::string> specialFields;
    int field = 0;
    while (true) {
        std::string key = "special_field_" + std::to_string(field);
        if (parameterMap.find(key) != parameterMap.end()) {
            specialFields.push_back(parameterMap[key]);
            ++field;
        } else {
            break;
        }
    }

    std::vector<std::string> strObjDensitiesVec = utility::splitByDelim(objDensities, ',');
    this->objDensities.clear();
    for (std::size_t i = 0; i < strObjDensitiesVec.size(); ++i) {
        double density = std::stod(strObjDensitiesVec[i]);
        if (density > 0) {
            this->objDensities.push_back(density);
        } else {
            std::cerr << "Invalid density in list provided!\n";
            exit(1);
        }
    }
    this->objTypes = utility::splitByDelim(objTypes, ',');
    std::vector<std::string> strKValuesVec = utility::splitByDelim(kValues, ',');
    std::vector<int> kValuesVec;
    for (std::size_t i = 0; i < strKValuesVec.size(); ++i) {
        int k = std::stoi(strKValuesVec[i]);
        if (k > 0) {
            kValuesVec.push_back(k);
        } else {
            std::cerr << "Invalid k value in list provided!\n";
            exit(1);
        }
    }
    std::vector<std::string> strObjVariables = utility::splitByDelim(objVariable, ',');
    this->objVariable.clear();
    for (std::size_t i = 0; i < strObjVariables.size(); ++i) {
        int variable = std::stoi(strObjVariables[i]);
        if (variable > 0) {
            this->objVariable.push_back(variable);
        } else {
            std::cerr << "Invalid variable in list provided (must be greater than zero)!\n";
            exit(1);
        }
    }
    if (this->objDensities.size() == 0 || this->objTypes.size() == 0 || this->objVariable.size() == 0 ||
        kValuesVec.size() == 0) {
        std::cerr << "Not enough densities or types provided!\n";
        exit(1);
    }

    std::string gtreeIdxFile, roadIdxFile, silcIdxFile, juncIdxFile, phlIdxFile, altIdxFile;
    std::vector<std::string> parameterKeys, parameterValues;

    this->kValues = kValuesVec;
    this->numSets = numSets;
    this->filePathPrefix = filePathPrefix;

    if (method == constants::INE_KNN_QUERY) {
        this->runINEQueries();
    } else if (method == constants::GTREE_KNN_QUERY) {
        int fanout = std::stoi(parameterMap["gtree_fanout"]);
        std::size_t maxLeafSize = std::stoi(parameterMap["gtree_maxleafsize"]);
        if (fanout < 2 || maxLeafSize < 32) {
            std::cerr << "Invalid Gtree parameters!\n";
            exit(1);
        }
        this->runGtreeQueries(fanout, maxLeafSize);
    } else if (method == constants::AGTREE_KNN_QUERY) {
        int fanout = std::stoi(parameterMap["gtree_fanout"]);
        std::size_t maxLeafSize = std::stoi(parameterMap["gtree_maxleafsize"]);
        if (fanout < 2 || maxLeafSize < 32) {
            std::cerr << "Invalid Gtree parameters!\n";
            exit(1);
        }
        this->runAGtreeQueries(fanout, maxLeafSize);
    } else if (method == constants::IER_DIJKSTRA_KNN_QUERY) {
        for (auto branchFactor: {8}) {
            std::cout << "Branch factor: " << branchFactor << std::endl;
            this->runIERQueries(branchFactor);
        }
    } else if (method == constants::IER_ASTAR_KNN_QUERY) {
        for (auto branchFactor: {8}) {
            std::cout << "Branch factor: " << branchFactor << std::endl;
            this->runIERAStarQueries(branchFactor);
        }
    }else if (method == constants::IER_ALT_KNN_QUERY) {
        int numLandmarks = std::stoi(parameterMap["landmarks"]);
        int landmarksType = std::stoi(parameterMap["landmarks_type"]);
        for (auto branchFactor: {8}) {
            std::cout << "Branch factor: " << branchFactor << std::endl;
            this->runIERALTQueries(branchFactor, numLandmarks, static_cast<LANDMARK_TYPE>(landmarksType));
        }
    } else {
        std::cerr << "Could not recognise method - check kNN query command" << std::endl;
        exit(1);
    }

    std::cout << "-------" << std::endl;
}

void AdaptiveExperimentsCommand::runExperiment(Experiment &experiment)
{
    std::vector<NodeID> kNNs, ineKNNs;
    std::vector<EdgeWeight> kNNDistances, ineKNNDistances;

    std::string objSetType;

    StopWatch sw;
    double totalQueryTime;
    int totalQueries = numSets * queryNodes.size();

    Statistics knnStats;
    std::string message;

    int queryCounter = 0;

    INEExperiment verifyExperiment;

    sw.start();
    experiment.buildIndex(graph);
    sw.stop();
    experiment.printInfo();
    std::cout << "Time to build index and query times" << std::endl << sw.getTimeMs() << std::endl;
    sw.reset();

//    std::cout << "numberOfQueries, totalQueryTime" << std::endl;

    for (std::size_t i = 0; i < objTypes.size(); ++i) {
        for (std::size_t j = 0; j < objDensities.size(); ++j) {
            for (std::size_t m = 0; m < objVariable.size(); ++m) {
                for (std::size_t k = 0; k < kValues.size(); ++k) {
                    totalQueryTime = 0;
#if defined(COLLECT_STATISTICS)
                    knnStats.clear();
#endif
                    for (std::size_t l = 0; l < numSets; ++l) {
                        std::string objSetFile = filePathPrefix + "/obj_indexes/" +
                                                 utility::constructObjsectSetFileName(graph.getNetworkName(),
                                                                                      objTypes[i], objDensities[j],
                                                                                      objVariable[m], l);
                        int setSize;
                        std::vector<NodeID> objectNodes = utility::getPointSetFromFile(objSetFile, objTypes[i],
                                                                                       objDensities[j], setSize,
                                                                                       objVariable[m]);
                        graph.resetAllObjects();
                        graph.parseObjectSet(objectNodes);

                        experiment.clearObjects();

                        StopWatch objectsSw;
                        objectsSw.start();
                        experiment.loadObjects(graph, objectNodes);
                        objectsSw.stop();
                        std::cout << "Time to load objects" << std::endl << objectsSw.getTimeMs() << std::endl;
                        std::cout << "Query times" << std::endl;

                        for (auto queryNodeIt = queryNodes.begin(); queryNodeIt != queryNodes.end(); ++queryNodeIt) {
                            kNNs.clear();
                            kNNDistances.clear();
                            kNNs.reserve(kValues[k]);
                            kNNDistances.reserve(kValues[k]);
                            sw.reset();
                            sw.start();
                            experiment.runQuery(graph, kValues[k], *queryNodeIt, kNNs, kNNDistances);
                            sw.stop();
                            totalQueryTime += sw.getTimeMs();
#if defined(COLLECT_STATISTICS)
                            //knnStats.mergeStatistics(ine.stats);
#endif
                            if (verifyKNN) {
                                ineKNNs.clear();
                                ineKNNDistances.clear();
                                verifyExperiment.clearObjects();
                                verifyExperiment.loadObjects(graph, objectNodes);
                                verifyExperiment.runQuery(graph, kValues[k], *queryNodeIt, ineKNNs, ineKNNDistances);
                                if (!utility::verifyKNN(ineKNNs, ineKNNDistances, kNNs, kNNDistances, false, kValues[k],
                                                        message, true)) {
                                    std::cout << "Verfication failed for query node " << *queryNodeIt << " with k = "
                                              << kValues[k]
                                              << std::endl;
                                    std::cout << "Message: " << message << std::endl;
                                    exit(1);
                                }
                            }
                            queryCounter++;
                            if (queryCounter == 5 || queryCounter == 10 || queryCounter == 100 ||
                                queryCounter == 1000 || queryCounter == 10000) {
                                std::cout << queryCounter << ", " << totalQueryTime << std::endl;
//                                std::cout << totalQueryTime << std::endl;
//                                if (queryCounter == 1000) {
//                                    break;
//                                }
                            }
                        }
                    }

                    double queryTimeMs = totalQueryTime / totalQueries;
#if defined(COLLECT_STATISTICS)
                    knnStats.normalizeStatistics(totalQueries);
#endif

                    // Collect stats and return to output to file
                    kNNs.clear(); // Clear so that we don't pass last executed queries results to stats tuple
                    kNNDistances.clear();
                }
            }
        }
    }
    experiment.printSummary();
    std::cout << experiment.getName() << " kNN queries successfully executed for " << graph.getNetworkName()
              << std::endl;
}

void AdaptiveExperimentsCommand::runINEQueries()
{
    INEExperiment ineExperiment;
    std::vector<std::string> tempVec;
    this->runExperiment(ineExperiment);
}

void
AdaptiveExperimentsCommand::runGtreeQueries(int fanout, int maxLeafSize)
{
    GTreeExperiment gtreeExperiment(fanout, maxLeafSize);
    this->runExperiment(gtreeExperiment);
}

void
AdaptiveExperimentsCommand::runAGtreeQueries(int fanout, int maxLeafSize)
{
    AdaptiveGTreeExperiment adaptiveGTreeExperiment(fanout, maxLeafSize);
    this->runExperiment(adaptiveGTreeExperiment);
}

void AdaptiveExperimentsCommand::runIERQueries(unsigned int branchFactor)
{
    IERExperiment ierExperiment(branchFactor);
    this->runExperiment(ierExperiment);
}

void AdaptiveExperimentsCommand::runIERAStarQueries(unsigned int branchFactor)
{
    IERAStarExperiment experiment(branchFactor);
    this->runExperiment(experiment);
}

void AdaptiveExperimentsCommand::runIERALTQueries(unsigned int branchFactor, unsigned int numLandmarks, LANDMARK_TYPE landmarkType)
{
    IERALTExperiment experiment(branchFactor, numLandmarks, landmarkType, ALTParameters::buildKnnParameters());
    this->runExperiment(experiment);
}
