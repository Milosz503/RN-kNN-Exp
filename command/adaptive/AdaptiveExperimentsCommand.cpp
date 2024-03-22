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

#include <cstdio>
#include <cmath>

static const int STATS_STEP = 100;

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

    /*
     * Process Command Line Arguments
     */
    int opt;
    while ((opt = getopt(argc, argv, "e:g:p:f:s:n:d:t:q:k:m:v:l:r:")) != -1) {
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
            default:
                std::cerr << "Unknown option(s) provided!\n\n";
                showCommandUsage(argv[0]);
                exit(1);
        }
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
    if (experiment == constants::EXP_RUN_KNN) {
        if (argc < 25) {
            // Arguments: -g <binary graph file> -q <query node file> -k <k values> -p <parameters> -n <num sets> -d <list of object densities> -t <list of object types>
            // -v <list of some object variable> -f <index output file path prefix> -s <stats output file>
            std::cerr << "Too few arguments!\n\n";
            this->showPhaseUsage(experiment, argv[0]);
            exit(1);
        }

        if (bgrFilePath == "" || parameters == "" || filePathPrefix == "" || statsOutputFile == ""
            || numSets == 0 || objDensities == "" || objTypes == "" || queryNodeFile == "" || kValues == "" ||
            objVariable == "") {
            std::cerr << "Invalid argument(s)!\n\n";
            this->showPhaseUsage(experiment, argv[0]);
            exit(1);
        }

        this->runQueries(bgrFilePath, queryNodeFile, kValues, parameters, numSets, objDensities, objTypes, objVariable,
                         filePathPrefix, statsOutputFile);

    } else if (experiment == constants::EXP_RUN_KNN_OPTIMIZATIONS) {
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
            std::cerr << "Invalid key-value pair in parameter string" << std::endl;
            exit(1);
        }
    }
    return parameterMap;
}


void AdaptiveExperimentsCommand::runSingleMethodQueries(std::string method, std::string bgrFileName,
                                                        std::string queryNodeFile, std::string kValues,
                                                        std::string parameters,
                                                        std::size_t numSets, std::string objDensities,
                                                        std::string objTypes, std::string objVariable,
                                                        std::string filePathPrefix, std::string statsOutputFile)
{
    Graph graph = serialization::getIndexFromBinaryFile<Graph>(bgrFileName);

    std::cout << "--- Running experiment, method: " << method << ", network: " << graph.getNetworkName() << " ---" << std::endl;

    std::unordered_map<std::string, std::string> parameterMap = this->getParameters(parameters);

    std::vector<NodeID> queryNodes = utility::getPointSetFromFile(queryNodeFile);

    bool verifykNN = parameterMap["verify"] == "1";

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
    std::vector<double> objDensitiesVec;
    for (std::size_t i = 0; i < strObjDensitiesVec.size(); ++i) {
        double density = std::stod(strObjDensitiesVec[i]);
        if (density > 0) {
            objDensitiesVec.push_back(density);
        } else {
            std::cerr << "Invalid density in list provided!\n";
            exit(1);
        }
    }
    std::vector<std::string> objTypesVec = utility::splitByDelim(objTypes, ',');
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
    std::vector<int> objVariableVec;
    for (std::size_t i = 0; i < strObjVariables.size(); ++i) {
        int variable = std::stoi(strObjVariables[i]);
        if (variable > 0) {
            objVariableVec.push_back(variable);
        } else {
            std::cerr << "Invalid variable in list provided (must be greater than zero)!\n";
            exit(1);
        }
    }
    if (objDensitiesVec.size() == 0 || objTypesVec.size() == 0 || objVariableVec.size() == 0 ||
        kValuesVec.size() == 0) {
        std::cerr << "Not enough densities or types provided!\n";
        exit(1);
    }

    std::string gtreeIdxFile, roadIdxFile, silcIdxFile, juncIdxFile, phlIdxFile, altIdxFile;
    std::vector<std::string> parameterKeys, parameterValues;

    if (method == constants::INE_KNN_QUERY) {
        this->runINEQueries(graph, queryNodes, kValuesVec, numSets, objDensitiesVec, objTypesVec, objVariableVec,
                            filePathPrefix, statsOutputFile, specialFields);
    } 
//    else if (method == "bad_ine") {
//        // Note: This is to compare INE when non-ideal data structures are chosen
//        std::string dynBgrFileName = filePathPrefix + "/indexes/" + graph.getNetworkName() + "_dynamic.bin";
//        this->runINEQueriesByDynamicGraph(graph, dynBgrFileName, queryNodes, kValuesVec, numSets, objDensitiesVec,
//                                          objTypesVec, objVariableVec, filePathPrefix, statsOutputFile, verifykNN,
//                                          specialFields);
//    } 
    else if (method == constants::GTREE_KNN_QUERY_BUILD) {
        int fanout = std::stoi(parameterMap["gtree_fanout"]);
        std::size_t maxLeafSize = std::stoi(parameterMap["gtree_maxleafsize"]);
        if (fanout < 2 || maxLeafSize < 32) {
            std::cerr << "Invalid Gtree parameters!\n";
            exit(1);
        }
        parameterKeys.clear();
        parameterValues.clear();
        parameterKeys.push_back("fanout");
        parameterValues.push_back(parameterMap["gtree_fanout"]);
        parameterKeys.push_back("maxleafsize");
        parameterValues.push_back(parameterMap["gtree_maxleafsize"]);
        gtreeIdxFile = filePathPrefix + "/indexes/" +
                       utility::constructIndexFileName(graph.getNetworkName(), constants::IDX_GTREE_CMD, parameterKeys,
                                                       parameterValues);
        this->runGtreeQueries(graph, gtreeIdxFile, queryNodes, kValuesVec, numSets, objDensitiesVec, objTypesVec,
                              objVariableVec, filePathPrefix, statsOutputFile, verifykNN, parameterKeys,
                              parameterValues, specialFields);
    } else if (method == constants::GTREE_KNN_QUERY) {
        int fanout = std::stoi(parameterMap["gtree_fanout"]);
        std::size_t maxLeafSize = std::stoi(parameterMap["gtree_maxleafsize"]);
        if (fanout < 2 || maxLeafSize < 32) {
            std::cerr << "Invalid Gtree parameters!\n";
            exit(1);
        }
        parameterKeys.clear();
        parameterValues.clear();
        parameterKeys.push_back("fanout");
        parameterValues.push_back(parameterMap["gtree_fanout"]);
        parameterKeys.push_back("maxleafsize");
        parameterValues.push_back(parameterMap["gtree_maxleafsize"]);
        gtreeIdxFile = filePathPrefix + "/indexes/" +
                       utility::constructIndexFileName(graph.getNetworkName(), constants::IDX_GTREE_CMD, parameterKeys,
                                                       parameterValues);
        this->runGtreeQueries(graph, gtreeIdxFile, queryNodes, kValuesVec, numSets, objDensitiesVec, objTypesVec,
                              objVariableVec, filePathPrefix, statsOutputFile, verifykNN, parameterKeys,
                              parameterValues, specialFields);
    } else if (method == constants::AGTREE_KNN_QUERY) {
        int fanout = std::stoi(parameterMap["gtree_fanout"]);
        std::size_t maxLeafSize = std::stoi(parameterMap["gtree_maxleafsize"]);
        if (fanout < 2 || maxLeafSize < 32) {
            std::cerr << "Invalid Gtree parameters!\n";
            exit(1);
        }
        parameterKeys.clear();
        parameterValues.clear();
        parameterKeys.push_back("fanout");
        parameterValues.push_back(parameterMap["gtree_fanout"]);
        parameterKeys.push_back("maxleafsize");
        parameterValues.push_back(parameterMap["gtree_maxleafsize"]);
        gtreeIdxFile = filePathPrefix + "/indexes/" +
                       utility::constructIndexFileName(graph.getNetworkName(), constants::IDX_AGTREE_CMD, parameterKeys,
                                                       parameterValues);
        this->runAGtreeQueries(graph, gtreeIdxFile, queryNodes, kValuesVec, numSets, objDensitiesVec, objTypesVec,
                               objVariableVec, filePathPrefix, statsOutputFile, verifykNN, parameterKeys,
                               parameterValues, specialFields);
    } else if (method == constants::IER_DIJKSTRA_KNN_QUERY) {
        for (auto branchFactor : {8, 16, 32}) {
            std::cout << "Branch factor: " << branchFactor << std::endl;
            parameterKeys.clear();
            parameterValues.clear();
            parameterKeys.push_back("branchfactor");
            parameterValues.push_back(std::to_string(branchFactor));
            this->runIERQueries(graph, queryNodes, kValuesVec, numSets, objDensitiesVec, objTypesVec,
                                objVariableVec, filePathPrefix, statsOutputFile, verifykNN, parameterKeys,
                                parameterValues, specialFields);
        }
    } else {
        std::cerr << "Could not recognise method - check kNN query command" << std::endl;
        exit(1);
    }

    std::cout << "-------" << std::endl;
}


void AdaptiveExperimentsCommand::runQueries(std::string bgrFileName, std::string queryNodeFile, std::string kValues,
                                            std::string parameters, std::size_t numSets, std::string objDensities,
                                            std::string objTypes, std::string objVariable, std::string filePathPrefix,
                                            std::string statsOutputFile)
{
    Graph graph = serialization::getIndexFromBinaryFile<Graph>(bgrFileName);
    std::unordered_map<std::string, std::string> parameterMap = this->getParameters(parameters);

    std::vector<NodeID> queryNodes = utility::getPointSetFromFile(queryNodeFile);
    if (queryNodes.size() == 0) {
        std::cerr << "No query points were provided!\n";
        exit(1);
    }
    if (numSets == 0) {
        std::cerr << "No object sets were provided!\n";
        exit(1);
    }

    bool verifykNN = parameterMap["verify"] == "1";
    bool queryINE = parameterMap["ine"] == "1";
    bool queryGtree = parameterMap["gtree"] == "1";
    bool queryRoad = parameterMap["road"] == "1";
    bool queryIER = parameterMap["ier"] == "1";
    bool querySILC = parameterMap["silc"] == "1";
    bool queryDistBrws = parameterMap["dist_brws"] == "1";
    bool queryIERPHL = parameterMap["ier_phl"] == "1";

    std::vector<std::string> strObjDensitiesVec = utility::splitByDelim(objDensities, ',');
    std::vector<double> objDensitiesVec;
    for (std::size_t i = 0; i < strObjDensitiesVec.size(); ++i) {
        double density = std::stod(strObjDensitiesVec[i]);
        if (density > 0) {
            objDensitiesVec.push_back(density);
        } else {
            std::cerr << "Invalid density in list provided!\n";
            exit(1);
        }
    }
    std::vector<std::string> objTypesVec = utility::splitByDelim(objTypes, ',');
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
    std::vector<int> objVariableVec;
    for (std::size_t i = 0; i < strObjVariables.size(); ++i) {
        int variable = std::stoi(strObjVariables[i]);
        if (variable > 0) {
            objVariableVec.push_back(variable);
        } else {
            std::cerr << "Invalid variable in list provided (must be greater than zero)!\n";
            exit(1);
        }
    }
    if (objDensitiesVec.size() == 0 || objTypesVec.size() == 0 || objVariableVec.size() == 0 ||
        kValuesVec.size() == 0) {
        std::cerr << "Not enough densities or types provided!\n";
        exit(1);
    }

    if (queryINE) {
        this->runINEQueries(graph, queryNodes, kValuesVec, numSets, objDensitiesVec, objTypesVec, objVariableVec,
                            filePathPrefix, statsOutputFile);
    }

    std::string gtreeIdxFile, roadIdxFile, silcIdxFile, juncIdxFile, phlIdxFile;
    std::vector<std::string> parameterKeys, parameterValues;

    if (queryGtree) {
        int fanout = std::stoi(parameterMap["gtree_fanout"]);
        std::size_t maxLeafSize = std::stoi(parameterMap["gtree_maxleafsize"]);
        if (fanout < 2 || maxLeafSize < 32) {
            std::cerr << "Invalid Gtree parameters!\n";
            exit(1);
        }
        parameterKeys.clear();
        parameterValues.clear();
        parameterKeys.push_back("fanout");
        parameterValues.push_back(parameterMap["gtree_fanout"]);
        parameterKeys.push_back("maxleafsize");
        parameterValues.push_back(parameterMap["gtree_maxleafsize"]);
        gtreeIdxFile = filePathPrefix + "/indexes/" +
                       utility::constructIndexFileName(graph.getNetworkName(), constants::IDX_GTREE_CMD, parameterKeys,
                                                       parameterValues);
        this->runGtreeQueries(graph, gtreeIdxFile, queryNodes, kValuesVec, numSets, objDensitiesVec, objTypesVec,
                              objVariableVec, filePathPrefix, statsOutputFile, verifykNN, parameterKeys,
                              parameterValues);
    }

    std::cout << "Query testing complete!" << std::endl;
}

void AdaptiveExperimentsCommand::runExperiment(Experiment &experiment, Graph &graph, std::vector<NodeID> &queryNodes,
                                               std::vector<int> &kValues, std::size_t numSets,
                                               std::vector<double> objDensities, std::vector<std::string> objTypes,
                                               std::vector<int> objVariable, std::string filePathPrefix,
                                               std::string statsOutputFile, bool verifyKNN,
                                               std::vector<std::string> &parameterKeys,
                                               std::vector<std::string> &parameterValues,
                                               std::vector<std::string> specialFields)
{
    std::vector<NodeID> kNNs, ineKNNs;
    std::vector<EdgeWeight> kNNDistances, ineKNNDistances;

    std::string objSetType;
    double objSetDensity;
    int objSetSize, objSetVariable;

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
                        experiment.loadObjects(graph, filePathPrefix, objTypes[i],objDensities[j],objVariable[m],l,
                                               parameterKeys, parameterValues);

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
                            knnStats.mergeStatistics(ine.stats);
#endif
                            if (verifyKNN) {
                                ineKNNs.clear();
                                ineKNNDistances.clear();
                                verifyExperiment.loadObjects(graph, filePathPrefix, objTypes[i],objDensities[j],objVariable[m],l,
                                                       parameterKeys, parameterValues);
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
                            if (queryCounter == 5 || queryCounter == 10 || queryCounter == 100 || queryCounter == 1000 || queryCounter == 10000) {
//                                std::cout << queryCounter << ", " << totalQueryTime << std::endl;
                                std::cout << totalQueryTime << std::endl;
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
                    KnnQueryTuple stats(graph.getNetworkName(), graph.getNumNodes(), graph.getNumEdges(), totalQueries,
                                        constants::INE_KNN_QUERY,
                                        kValues[k], queryTimeMs, objTypes[i], objDensities[j], objVariable[m],
                                        0, kNNs, kNNDistances);
                    stats.setAdditionalFields(specialFields);
#if defined(COLLECT_STATISTICS)
                    knnStats.populateTupleFields(stats,0);
#endif
                    this->outputCommandStats(statsOutputFile, stats.getTupleString());
                }
            }
        }
    }
    std::cout << experiment.getName() << " kNN queries successfully executed for " << graph.getNetworkName()
              << std::endl;
}

void AdaptiveExperimentsCommand::runINEQueries(Graph &graph, std::vector<NodeID> &queryNodes, std::vector<int> &kValues,
                                               std::size_t numSets,
                                               std::vector<double> objDensities, std::vector<std::string> objTypes,
                                               std::vector<int> objVariable, std::string filePathPrefix,
                                               std::string statsOutputFile, std::vector<std::string> specialFields)
{
    INEExperiment ineExperiment;
    std::vector<std::string> tempVec;
    this->runExperiment(ineExperiment, graph, queryNodes, kValues, numSets, objDensities, objTypes, objVariable,
                       filePathPrefix, statsOutputFile, false, specialFields, tempVec, tempVec);
}



// TODO: implement INE queries by dynamic graph
//void AdaptiveExperimentsCommand::runINEQueriesByDynamicGraph(Graph &graph, std::string dynBgrFileName,
//                                                             std::vector<NodeID> &queryNodes, std::vector<int> &kValues,
//                                                             std::size_t numSets,
//                                                             std::vector<double> objDensities,
//                                                             std::vector<std::string> objTypes,
//                                                             std::vector<int> objVariable, std::string filePathPrefix,
//                                                             std::string statsOutputFile, bool verifyKNN,
//                                                             std::vector<std::string> specialFields)
//{
//
//}


void
AdaptiveExperimentsCommand::runGtreeQueries(Graph &graph, std::string gtreeIdxFile, std::vector<NodeID> &queryNodes,
                                            std::vector<int> &kValues, std::size_t numSets,
                                            std::vector<double> objDensities,
                                            std::vector<std::string> objTypes, std::vector<int> objVariable,
                                            std::string filePathPrefix, std::string statsOutputFile,
                                            bool verifyKNN, std::vector<std::string> &parameterKeys,
                                            std::vector<std::string> &parameterValues,
                                            std::vector<std::string> specialFields)
{
    GTreeExperiment gtreeExperiment(std::stoi(parameterValues[0]), std::stoi(parameterValues[1]));
    this->runExperiment(gtreeExperiment, graph, queryNodes, kValues, numSets, objDensities, objTypes, objVariable,
                        filePathPrefix, statsOutputFile, verifyKNN, parameterKeys, parameterValues, specialFields);
}

void
AdaptiveExperimentsCommand::runAGtreeQueries(Graph &graph, std::string gtreeIdxFile, std::vector<NodeID> &queryNodes,
                                             std::vector<int> &kValues, std::size_t numSets,
                                             std::vector<double> objDensities,
                                             std::vector<std::string> objTypes, std::vector<int> objVariable,
                                             std::string filePathPrefix, std::string statsOutputFile,
                                             bool verifyKNN, std::vector<std::string> &parameterKeys,
                                             std::vector<std::string> &parameterValues,
                                             std::vector<std::string> specialFields)
{
    AdaptiveGTreeExperiment adaptiveGTreeExperiment(std::stoi(parameterValues[0]),
                                                    std::stoi(parameterValues[1]));
    this->runExperiment(adaptiveGTreeExperiment, graph, queryNodes, kValues, numSets, objDensities, objTypes, objVariable,
                        filePathPrefix, statsOutputFile, verifyKNN, parameterKeys, parameterValues, specialFields);
}

void AdaptiveExperimentsCommand::runIERQueries(Graph &graph, std::vector<NodeID> &queryNodes,
                                               std::vector<int> &kValues, std::size_t numSets,
                                               std::vector<double> objDensities, std::vector<std::string> objTypes,
                                               std::vector<int> objVariable, std::string filePathPrefix,
                                               std::string statsOutputFile, bool verifyKNN, std::vector<std::string> &parameterKeys,
                                               std::vector<std::string> &parameterValues,
                                               std::vector<std::string> specialFields)
{
    IERExperiment ierExperiment;
    this->runExperiment(ierExperiment, graph, queryNodes, kValues, numSets, objDensities, objTypes, objVariable,
                        filePathPrefix, statsOutputFile, verifyKNN, parameterKeys, parameterValues, specialFields);
}
