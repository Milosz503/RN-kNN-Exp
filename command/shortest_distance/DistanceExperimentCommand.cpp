//
// Created by milos on 21/03/2024.
//

#include "DistanceExperimentCommand.h"
#include "../../utility/serialization.h"
#include "../../utility/StopWatch.h"
#include "QueryGenerator.h"
#include "../../utils/Utils.h"
#include <fstream>

std::unordered_map<std::string, std::unordered_map<std::string, double>> gtreeConfig = {
        {"DE", {{"fanout", 4}, {"maxleafsize", 64}}},
        {"ME", {{"fanout", 4}, {"maxleafsize", 128}}},
        {"NW", {{"fanout", 4}, {"maxleafsize", 256}}},
        {"W", {{"fanout", 4}, {"maxleafsize", 512}}},
        {"USA", {{"fanout", 4}, {"maxleafsize", 512}}}
};

std::vector<std::tuple<int, double>> estConfigs = {
        {6,  0.32},
        {8,  0.26},
        {12, 0.2},
        {20, 0.16},
        {36, 0.12}
};

std::vector<std::tuple<int, double>> distConfigs = {
        {6,  0.47},
        {8,  0.39},
        {12, 0.33},
        {20, 0.26},
        {36, 0.2}
};

std::vector<std::tuple<int, double>> hopsConfigs = {
        {6,  0.4},
        {8,  0.35},
        {12, 0.27},
        {20, 0.2},
        {36, 0.15}
};


void DistanceExperimentCommand::compareOtherMethods()
{
    if (gtreeConfig.find(network) == gtreeConfig.end()) {
        std::cerr << "No gtree configuration found for network: " << network << std::endl;
        exit(1);
    }
    auto gtreeParams = gtreeConfig[network];
    methods.push_back(new AdaptiveGtreeMethod(gtreeParams["fanout"], gtreeParams["maxleafsize"]));
    methods.push_back(new GtreeMethod(gtreeParams["fanout"], gtreeParams["maxleafsize"]));
    methods.push_back(new DijkstraMethod());
    methods.push_back(new AStarMethod());
    methods.push_back(new PhlMethod());

    results.resize(methods.size());

    for (int i = 0; i < numRepeats; i++) {
        std::cout << "Repeat: " << i << std::endl;
        buildIndexes();
        loadQueries();
        runAll();
        validateAll();
        queries.clear();
    }
}

void DistanceExperimentCommand::compareAdaptiveEstThresholdQueryVsTime()
{
    for (auto parameters: estConfigs) {
        methods.push_back(
                new AdaptiveALTMethod(
                        AdaptiveALTParams(
                                std::get<0>(parameters),
                                0.0, 0.0, 1.0, std::get<1>(parameters)
                        )
                ));
    }
    results.resize(methods.size());

    for (int i = 0; i < numRepeats; i++) {
        std::cout << "Repeat: " << i << std::endl;
        buildIndexes();
        loadQueries();
        runAll();
        queries.clear();
    }
}

void DistanceExperimentCommand::compareAdaptiveDistThresholdQueryVsTime()
{
    for (auto parameters: distConfigs) {
        methods.push_back(
                new AdaptiveALTMethod(
                        AdaptiveALTParams(
                                std::get<0>(parameters),
                                1.0, 0.0, 0.0, std::get<1>(parameters)
                        )
                ));
    }
    results.resize(methods.size());

    for (int i = 0; i < numRepeats; i++) {
        std::cout << "Repeat: " << i << std::endl;
        buildIndexes();
        loadQueries();
        runAll();
        queries.clear();
    }
}

void DistanceExperimentCommand::compareAdaptiveHopsThresholdQueryVsTime()
{
    for (auto parameters: distConfigs) {
        methods.push_back(
                new AdaptiveALTMethod(
                        AdaptiveALTParams(
                                std::get<0>(parameters),
                                0.0, 1.0, 0.0, std::get<1>(parameters)
                        )
                ));
    }
    results.resize(methods.size());

    for (int i = 0; i < numRepeats; i++) {
        std::cout << "Repeat: " << i << std::endl;
        buildIndexes();
        loadQueries();
        runAll();
        queries.clear();
    }
}

void DistanceExperimentCommand::compareAltEstLandmarksVsThreshold()
{

    std::vector<double> values;
    for (unsigned i = 120; i < 500; i += 10) {
        auto threshold = i / 1000.0;
        values.push_back(threshold);
    }

    std::vector<EdgeWeight> distances(numTargets, 0);
    results.resize(values.size());
    for (int i = 0; i < numRepeats; i++) {
        std::cout << "Repeat: " << i << std::endl;
        loadQueries();
        std::vector<std::string> methodNames;
        for (int j = 0; j < values.size(); j++) {
            auto params = AdaptiveALTParams(
                    150,
                    0.0, 0.0, 1.0, values[j]
            );
            auto method = AdaptiveALTMethod(params);
            methodNames.push_back(method.getInfo());
            method.buildIndex(graph);
            std::cout << "Running method: " << method.getInfo() << std::endl;
            for (unsigned k = 0; k < queries.size(); ++k) {
                method.refineIndex(graph, queries[k], distances);
            }
            method.printStatistics();
            std::cout << std::endl;
            results[j].push_back(method.getLandmarkNodeIDs().size());
        }

        // Write after each repeat in case program crashes, do not average results
        write_to_csv(results, methodNames, getResultsPath(), 1);
    }
}

void DistanceExperimentCommand::compareAltHopsLandmarksVsThreshold()
{

    std::vector<double> values;
    for (unsigned i = 150; i < 500; i += 10) {
        auto threshold = i / 1000.0;
        values.push_back(threshold);
    }

    results.resize(values.size());
    for (int i = 0; i < numRepeats; i++) {
        std::cout << "Repeat: " << i << std::endl;
        std::vector<std::string> methodNames;
        for (int j = 0; j < values.size(); j++) {
            auto method = ALTMethod(100,
                                    LANDMARK_TYPE::HOPS,
                                    ALTParameters(values[j], numQueries));
            methodNames.push_back(method.getInfo());
            method.buildIndex(graph);
            results[j].push_back(method.getLandmarkNodeIDs().size());
        }

        // Write after each repeat in case program crashes, do not average results
        write_to_csv(results, methodNames, getResultsPath(), 1);
    }
}

void DistanceExperimentCommand::compareAltDistLandmarksVsThreshold()
{

    std::vector<double> values;
    for (unsigned i = 150; i < 500; i += 10) {
        auto threshold = i / 1000.0;
        values.push_back(threshold);
    }

    results.resize(values.size());
    for (int i = 0; i < numRepeats; i++) {
        std::cout << "Repeat: " << i << std::endl;
        std::vector<std::string> methodNames;
        for (int j = 0; j < values.size(); j++) {
            auto method = ALTMethod(100,
                                    LANDMARK_TYPE::MIN_DIST,
                                    ALTParameters(values[j], numQueries));
            methodNames.push_back(method.getInfo());
            method.buildIndex(graph);
            results[j].push_back(method.getLandmarkNodeIDs().size());
        }

        // Write after each repeat in case program crashes, do not average results
        write_to_csv(results, methodNames, getResultsPath(), 1);
    }
}

void DistanceExperimentCommand::visualizeQueries()
{
    queries = QueryGenerator().randomExpandTargetsClustered(graph, numQueries, 3, 0.0001);
    saveQueries("clustered");

    queries = QueryGenerator().random(graph, numQueries, numTargets, std::numeric_limits<unsigned long>::max());
    saveQueries("random");
}

void DistanceExperimentCommand::visualizeLandmarks()
{
    methods.push_back(new ALTMethod(15, LANDMARK_TYPE::FARTHEST, {}));
    methods.push_back(new ALTMethod(15, LANDMARK_TYPE::AVOID, {}));
    methods.push_back(new ALTMethod(15, LANDMARK_TYPE::MIN_DIST, ALTParameters(0.28, numQueries)));
    methods.push_back(new ALTMethod(15, LANDMARK_TYPE::HOPS, ALTParameters(0.22, numQueries)));
    results.resize(methods.size());

    buildIndexes(false);
    exportLandmarks(network);
}

void DistanceExperimentCommand::compareLandmarksNumberVsTime()
{
    for (int i = 2; i <= 40; i += 2) {
        methods.push_back(new ALTMethod(i, LANDMARK_TYPE::FARTHEST, {}));
    }
    results.resize(methods.size());

    for (unsigned i = 0; i < numRepeats; ++i) {
        std::cout << "Repeat: " << i << std::endl;
        buildIndexes(false);
        loadQueries();
        runAll(ResultsType::LAST);
    }

    queries.clear();
}

void DistanceExperimentCommand::compareFarthestALT()
{
    std::vector<unsigned> config = {
            6, 8, 12, 20, 36
    };

    for (auto landmarksNum: config) {
        methods.push_back(new ALTMethod(
                landmarksNum,
                LANDMARK_TYPE::FARTHEST,
                ALTParameters()
        ));
    }
    results.resize(methods.size());

    for (int i = 0; i < numRepeats; i++) {
        std::cout << "Repeat: " << i << std::endl;
        buildIndexes();
        loadQueries();
        runAll();
        queries.clear();
    }
}

void DistanceExperimentCommand::compareAvoidALT()
{
    std::vector<unsigned> config = {
            6, 8, 12, 20, 36
    };

    for (auto landmarksNum: config) {
        methods.push_back(new ALTMethod(
                landmarksNum,
                LANDMARK_TYPE::AVOID,
                ALTParameters()
        ));
    }
    results.resize(methods.size());

    for (int i = 0; i < numRepeats; i++) {
        std::cout << "Repeat: " << i << std::endl;
        buildIndexes();
        loadQueries();
        runAll();
        queries.clear();
    }
}

void DistanceExperimentCommand::compareAltEstThresholdQueryVsLandmarks()
{
    std::vector<double> values = {
            0.12, 0.16, 0.20, 0.24, 0.28, 0.32
    };

    results.resize(values.size());
    for (unsigned i = 0; i < values.size(); ++i) {
        auto params = AdaptiveALTParams(
                150,
                0.0, 0.0, 1.0, values[i]
        );
        params.results = &results[i];
        methods.push_back(new AdaptiveALTMethod(params));
    }

    for (int i = 0; i < numRepeats; i++) {
        std::cout << "Repeat: " << i << std::endl;
        buildIndexes(false);
        loadQueries();
        runAll(ResultsType::NONE);
    }

    // Do not average results, doesn't make sense
    write_to_csv(results, methods, getResultsPath(), 1);
}


void DistanceExperimentCommand::compareAltDistThresholdQueryVsLandmarks()
{
    std::vector<double> values = {
            0.16, 0.20, 0.24, 0.28, 0.32, 0.36
    };

    results.resize(values.size());
    for (unsigned i = 0; i < values.size(); ++i) {
        methods.push_back(new ALTMethod(
                150,
                LANDMARK_TYPE::MIN_DIST,
                ALTParameters(values[i], numQueries, &results[i])
        ));
    }

    for (int i = 0; i < numRepeats; i++) {
        std::cout << "Repeat: " << i << std::endl;
        buildIndexes(false);
    }

    // Do not average results, doesn't make sense
    write_to_csv(results, methods, getResultsPath(), 1);
}

void DistanceExperimentCommand::compareAltDistThresholdQueryVsTime()
{

    for (auto parameters: distConfigs) {
        methods.push_back(new ALTMethod(
                std::get<0>(parameters),
                LANDMARK_TYPE::MIN_DIST,
                ALTParameters(std::get<1>(parameters), numQueries)
        ));
    }
    results.resize(methods.size());

    for (int i = 0; i < numRepeats; i++) {
        std::cout << "Repeat: " << i << std::endl;
        buildIndexes();
        loadQueries();
        runAll();
        queries.clear();
    }
}

void DistanceExperimentCommand::compareAltHopsThresholdQueryVsLandmarks()
{
    std::vector<double> values = {
            0.15, 0.20, 0.25, 0.30, 0.35, 0.40
    };
    results.resize(values.size());
    for (unsigned i = 0; i < values.size(); ++i) {
        methods.push_back(new ALTMethod(
                150,
                LANDMARK_TYPE::HOPS,
                ALTParameters(values[i], numQueries, &results[i])
        ));
    }

    for (int i = 0; i < numRepeats; i++) {
        std::cout << "Repeat: " << i << std::endl;
        buildIndexes(false);
    }

    // Do not average results, doesn't make sense
    write_to_csv(results, methods, getResultsPath(), 1);
}

void DistanceExperimentCommand::compareAltHopsThresholdQueryVsTime()
{
    for (auto parameters: hopsConfigs) {
        methods.push_back(new ALTMethod(
                std::get<0>(parameters),
                LANDMARK_TYPE::HOPS,
                ALTParameters(std::get<1>(parameters), numQueries)
        ));
    }
    results.resize(methods.size());

    for (int i = 0; i < numRepeats; i++) {
        std::cout << "Repeat: " << i << std::endl;
        buildIndexes();
        loadQueries();
        runAll();
        queries.clear();
    }
}

void DistanceExperimentCommand::createMethodsConstABestThreshold()
{
    for (int i = 0; i < 11; i++) {
        methods.push_back(new AdaptiveALTMethod(AdaptiveALTParams(20, 0.0, i / 10.0, (10 - i) / 10.0, [](unsigned q) {
            return 0.15 * (pow(0.69, 0.0025 * q)) + 0.15;
        })));
    }
    results.resize(methods.size());

    for (int i = 0; i < numRepeats; i++) {
        std::cout << "Repeat: " << i << std::endl;
        buildIndexes();
        loadQueries();
        runAll();
        queries.clear();


        clearMethods();
        results.clear();
    }
}

void DistanceExperimentCommand::createMethodsConstBBestThreshold()
{
    for (int i = 0; i < 11; i++) {
        methods.push_back(new AdaptiveALTMethod(AdaptiveALTParams(20, i / 10.0, 0.0, (10 - i) / 10.0, [](unsigned q) {
            return 0.15 * (pow(0.69, 0.0025 * q)) + 0.15;
        })));
    }
    results.resize(methods.size());

    for (int i = 0; i < numRepeats; i++) {
        std::cout << "Repeat: " << i << std::endl;
        buildIndexes();
        loadQueries();
        runAll();
        queries.clear();
    }
}

void DistanceExperimentCommand::createMethodsConstCBestThreshold()
{
    for (int i = 0; i < 11; i++) {
        methods.push_back(new AdaptiveALTMethod(AdaptiveALTParams(20, i / 10.0, (10 - i) / 10.0, 0.0, [](unsigned q) {
            return 0.15 * (pow(0.69, 0.0025 * q)) + 0.15;
        })));
    }
    results.resize(methods.size());

    for (int i = 0; i < numRepeats; i++) {
        std::cout << "Repeat: " << i << std::endl;
        buildIndexes();
        loadQueries();
        runAll();
        queries.clear();
    }
}


void DistanceExperimentCommand::compareDecayFunctions()
{
    methods.push_back(new AdaptiveALTMethod(
            AdaptiveALTParams(20, 0.0, 1.0, 0.0, [](unsigned q) { return 0.15 * (pow(0.69, 0.0025 * q)) + 0.15; })));
    methods.push_back(new AdaptiveALTMethod(AdaptiveALTParams(20, 0.0, 1.0, 0.0, func)));
    methods.push_back(new AdaptiveALTMethod(AdaptiveALTParams(20, 0.0, 1.0, 0.0, exp_func)));
    methods.push_back(new AdaptiveALTMethod(AdaptiveALTParams(25, 0.0, 1.0, 0.0, exp_func)));
    results.resize(methods.size());

    for (int i = 0; i < numRepeats; i++) {
        std::cout << "Repeat: " << i << std::endl;
        buildIndexes();
        loadQueries();
        runAll();
        queries.clear();
    }
}


void DistanceExperimentCommand::compareThresholdLandmarkAdaptive(int numRepeats)
{
    for (double j = 0.1; j < 0.5; j += 0.03) {
        for (int i = 5; i < 41; i += 5) {
            methods.push_back(new AdaptiveALTMethod(AdaptiveALTParams(i, 0.0, 1.0, 0.0, j)));
        }
        results.resize(methods.size());
        for (int i = 0; i < numRepeats; i++) {
            std::cout << "Repeat: " << i << std::endl;
            buildIndexes();
            loadQueries();
            runAll();
            queries.clear();
        }
        write_to_csv(results, methods, network + "_adapt_" + std::to_string(j) + "_threshold_var_landmark", numRepeats);
        clearMethods();
        results.clear();
    }
}


void DistanceExperimentCommand::compareThresholdLandmark(int numRepeats, LANDMARK_TYPE landmarkType)
{
    for (double j = 0.1; j < 0.5; j += 0.03) {
        for (int i = 5; i < 41; i += 5) {
            methods.push_back(new ALTMethod(i, landmarkType, {j}));
        }
        std::vector<std::vector<double>> results(methods.size());
        for (int i = 0; i < numRepeats; i++) {
            std::cout << "Repeat: " << i << std::endl;
            buildIndexes();
            loadQueries();
            runAll();
            queries.clear();
        }
        write_to_csv(results, methods, network + "_nonadapt_" + std::to_string(j) + "_threshold_var_landmark_" +
                                       std::to_string(landmarkType) + "_type", numRepeats);
        clearMethods();
        results.clear();
    }
}


void DistanceExperimentCommand::compareMethods()
{
    methods.push_back(new ALTMethod(25, LANDMARK_TYPE::RANDOM, {0.12}));
    methods.push_back(new ALTMethod(20, LANDMARK_TYPE::RANDOM, {0.18}));
    methods.push_back(new ALTMethod(15, LANDMARK_TYPE::RANDOM, {0.24}));
    methods.push_back(new ALTMethod(25, LANDMARK_TYPE::AVOID, {0.12}));
    methods.push_back(new ALTMethod(20, LANDMARK_TYPE::AVOID, {0.18}));
    methods.push_back(new ALTMethod(15, LANDMARK_TYPE::AVOID, {0.24}));
    methods.push_back(new ALTMethod(25, LANDMARK_TYPE::MIN_DIST, {0.12}));
    methods.push_back(new ALTMethod(20, LANDMARK_TYPE::MIN_DIST, {0.18}));
    methods.push_back(new ALTMethod(15, LANDMARK_TYPE::MIN_DIST, {0.24}));
    methods.push_back(new AdaptiveALTMethod(AdaptiveALTParams(25, 0.0, 1.0, 0.0, 0.12)));
    methods.push_back(new AdaptiveALTMethod(AdaptiveALTParams(20, 0.0, 1.0, 0.0, 0.18)));
    methods.push_back(new AdaptiveALTMethod(AdaptiveALTParams(15, 0.0, 1.0, 0.0, 0.24)));
    methods.push_back(new AdaptiveALTMethod(
            AdaptiveALTParams(20, 0.0, 1.0, 0.0, [](unsigned q) { return 0.15 * (pow(0.69, 0.0025 * q)) + 0.15; })));
    std::vector<std::vector<double>> results(methods.size());

    for (int i = 0; i < numRepeats; i++) {
        std::cout << "Repeat: " << i << std::endl;
        buildIndexes();
        loadQueries();
        saveQueries(network + "_compare_methods");
        runAll();
        exportLandmarks(network + "_compare_methods");
        queries.clear();
    }
    write_to_csv(results, methods, network + "_compare_methods", numRepeats);
    clearMethods();
    results.clear();
}


void DistanceExperimentCommand::execute(int argc, char **argv)
{
    srand(7); // NOLINT(*-msc51-cpp) it is supposed to be possible to reproduce

    std::string bgrFilePath;
    LANDMARK_TYPE landmarkType = LANDMARK_TYPE::MIN_DIST;
    int testNum = -1;
    int opt;
    while ((opt = getopt(argc, argv, "e:g:p:f:s:n:d:t:q:k:m:v:l:r:t:x:w:")) != -1) {
        switch (opt) {
            case 'g':
                bgrFilePath = optarg;
                graphPath = bgrFilePath;
                break;
            case 'q':
                numQueries = std::stoul(optarg);
                break;
            case 'd':
                maxDist = std::stoul(optarg);
                break;
            case 'k':
                numTargets = std::stoul(optarg);
                break;
            case 't':
                landmarkType = static_cast<LANDMARK_TYPE>(std::stoi(optarg));
                break;
            case 'x':
                testNum = std::stoi(optarg);
                break;
            case 'r':
                numRepeats = std::stoi(optarg);
                break;
            case 'v':
                validate = std::stoi(optarg);
                break;
            case 'f':
                resultsPath = std::string(optarg);
                break;
            case 'w':
                workloadType = static_cast<WorkloadType>(std::stoi(optarg));
                break;
            default:
                std::cerr << "Unknown option(s) provided!\n\n";
//                std::cout << "Provided option: " << opt << std::endl;
//                showCommandUsage(argv[0]);
//                exit(1);
        }
    }
    if (numQueries == 0 || bgrFilePath.empty() || numTargets == 0) {
        std::cerr << "Missing required arguments!\n\n";
        showCommandUsage(argv[0]);
        exit(1);
    }

    auto indexSlash = bgrFilePath.find_last_of('/');
    auto indexDot = bgrFilePath.find_last_of('.');
    network = bgrFilePath.substr(indexSlash + 1, indexDot - indexSlash - 1);
    dataPath = bgrFilePath.substr(0, indexSlash);
    tsvPath = dataPath + "/" + network + ".tsv";

    std::cout << "** " << network << " **" << std::endl;
    if (!validate) {
        std::cout << "WARNING! Validation is disabled!" << std::endl;
    }
    std::cout << "Number of repeats: " << numRepeats << std::endl;
    switch (workloadType) {
        case WorkloadType::RANDOM:
            std::cout << "Workload type: Random" << std::endl;
            break;
        case WorkloadType::CLUSTER:
            std::cout << "Workload type: Clustered" << std::endl;
            break;
        default:
            std::cout << "Unknown workload type!" << std::endl;
            exit(1);
            break;
    }

    graph = serialization::getIndexFromBinaryFile<Graph>(bgrFilePath);


    switch (testNum) {
        case 0:
            runStandardTestCase([this] {
                createMethodsConstABestThreshold();
            });
            break;
        case 1:
            runStandardTestCase([this] {
                createMethodsConstBBestThreshold();
            });
            break;
        case 2:
            runStandardTestCase([this] {
                createMethodsConstCBestThreshold();
            });
            break;
        case 3:
            runStandardTestCase([this] {
                compareDecayFunctions();
            });
            break;
        case 4:
            compareThresholdLandmarkAdaptive(numRepeats);
            break;
        case 5:
            compareThresholdLandmark(numRepeats, landmarkType);
            break;
        case 6:
            runStandardTestCase([this] {
                compareLandmarksNumberVsTime();
            });
            break;
        case 7:
            compareAltDistThresholdQueryVsLandmarks();
            break;
        case 8:
            runStandardTestCase([this] {
                compareAltDistThresholdQueryVsTime();
            });
            break;
        case 9:
            compareAltHopsThresholdQueryVsLandmarks();
            break;
        case 10:
            runStandardTestCase([this] {
                compareAltHopsThresholdQueryVsTime();
            });
            break;
        case 11:
            runStandardTestCase([this] {
                compareFarthestALT();
            });
            break;
        case 12:
            runStandardTestCase([this] {
                compareAvoidALT();
            });
            break;
        case 13:
            visualizeLandmarks();
            break;
        case 14:
            visualizeQueries();
            break;
        case 15:
            compareAltDistLandmarksVsThreshold();
            break;
        case 16:
            compareAltHopsLandmarksVsThreshold();
            break;
        case 17:
            runStandardTestCase([this] {
                compareAdaptiveDistThresholdQueryVsTime();
            });
            break;
        case 18:
            runStandardTestCase([this] {
                compareAdaptiveHopsThresholdQueryVsTime();
            });
            break;
        case 19:
            runStandardTestCase([this] {
                compareOtherMethods();
            });
            break;
        case 20:
            compareAltEstThresholdQueryVsLandmarks();
            break;
        case 21:
            compareAltEstLandmarksVsThreshold();
            break;
        case 22:
            runStandardTestCase([this] {
                compareAdaptiveEstThresholdQueryVsTime();
            });
            break;
        default:
            compareMethods();
    }

    std::cout << "Finished" << std::endl;
}

void DistanceExperimentCommand::runStandardTestCase(const std::function<void()> &testCase)
{
    results.clear();
    clearMethods();

    testCase();

    write_to_csv(results, methods, getResultsPath(), numRepeats);
    results.clear();
    clearMethods();
}

void DistanceExperimentCommand::saveQueries(std::string name = "")
{
    std::ofstream file(resultsPath + "/" + name + "_query_data.csv");
    Coordinate x, y;
    file << "x,y,is_source,is_target" << std::endl;
    for (auto query: queries) {
        auto source = query.source;
        auto target = query.targets[0];

        graph.getCoordinates(source, x, y);
        file << x << ',' << y << ',' << true << ',' << false << std::endl;

        graph.getCoordinates(target, x, y);
        file << x << ',' << y << ',' << false << ',' << true << std::endl;
    }
    file.close();
}

void DistanceExperimentCommand::exportLandmarks(std::string name = "")
{
    size_t max_size = 0;
    for (const auto &method: methods) {
        max_size = std::max(max_size, method->getLandmarkNodeIDs().size());
    }

    std::vector<std::vector<std::string>> csvData(max_size, std::vector<std::string>(methods.size(), ""));

    Coordinate x, y;
    for (size_t i = 0; i < methods.size(); i++) {
        auto landmarks = methods[i]->getLandmarkNodeIDs();
        for (size_t j = 0; j < landmarks.size(); j++) {
            graph.getCoordinates(landmarks[j], x, y);
            csvData[j][i] = std::to_string(x) + "_" + std::to_string(y);
        }
    }
    std::ofstream file(resultsPath + "/" + name + "_landmark_data.csv");
    if (file.is_open()) {
        for (size_t i = 0; i < methods.size(); ++i) {
            file << methods[i]->getInfo();
            if (i < methods.size() - 1) {
                file << ",";
            }
        }
        file << std::endl;
        for (size_t i = 0; i < max_size; ++i) {
            for (size_t j = 0; j < methods.size(); ++j) {
                file << csvData[i][j];
                if (j < methods.size() - 1) {
                    file << ",";
                }
            }
            file << std::endl;
        }
        file.close();
        std::cout << "CSV file written successfully." << std::endl;
    } else {
        std::cerr << "Error opening output file." << std::endl;
    }
}

void DistanceExperimentCommand::showCommandUsage(std::string programName)
{
    std::cerr << "Usage: " << programName << " -c " + constants::DISTANCE_EXPERIMENTS_CMD +
                                             " -g <graph_file_name>" +
                                             " -q <num_of_queries>" +
                                             " -k <k>" +
                                             " \n\n";
}

void DistanceExperimentCommand::buildIndexes(bool saveTimes)
{
    int iter = 0;
    for (auto method: methods) {
        double time;
        if (method->getName() == "PHL") {
            time = dynamic_cast<PhlMethod *>(method)->buildIndex(graph, tsvPath);
        } else {
            StopWatch sw;
            sw.start();
            method->buildIndex(graph);
            sw.stop();
            time = sw.getTimeMs();
        }

        if (saveTimes) {
            results[iter].push_back(time);
        }
        iter++;
        std::cout << "Time to generate " << method->getName() << " index" << ": " << time << " ms"
                  << std::endl;
    }

}

void DistanceExperimentCommand::loadQueries()
{
    switch (workloadType) {
        case WorkloadType::RANDOM:
            queries = QueryGenerator().random(graph, numQueries, numTargets, std::numeric_limits<unsigned long>::max());
            break;
        case WorkloadType::CLUSTER:
            queries = QueryGenerator().randomExpandTargetsClustered(
                    graph, numQueries, 3, 100.0 / graph.getNumNodes());
            break;
        default:
            std::cout << "Unknown workload type!" << std::endl;
            exit(1);
            break;
    }
//    queries = QueryGenerator().randomExpandTargets(graph, numQueries, 0.000005);
//    queries = QueryGenerator().randomExpandTargetsClustered(graph, numQueries, 3, 0.0001);

}

void DistanceExperimentCommand::runAll(ResultsType resultsType)
{
    int iter = 0;
    for (auto method: methods) {
        runMethod(method, iter, resultsType);
        iter++;
    }
}

void DistanceExperimentCommand::validateAll()
{
    if (!validate) {
        std::cout << "WARNING: Skipping validation!" << std::endl;
        return;
    }
    std::cout << "*** Validating ***" << std::endl;
//    std::cout << "Validating edge weights..." << std::endl;
//
//    for (auto node: graph.getNodesIDsVector()) {
//        auto edgeStart = graph.getEdgeListStartIndex(node);
//        auto edgeEnd = graph.getEdgeListSize(node);
//
//        for (auto edge = edgeStart; edge < edgeEnd; edge++) {
//            auto edgeNode = graph.edges[edge].first;
//            auto edgeWeight = graph.edges[edge].second;
//            auto euclideanDist = graph.getEuclideanDistance(node, edgeNode);
//
//            if (euclideanDist > edgeWeight) {
//                std::cerr << "Validation failed for edge: " << node << " -> " << edgeNode << std::endl;
//                std::cerr << "Euclidean: " << euclideanDist << ", Edge: " << edgeWeight << std::endl;
//            }
//        }
//    }
//    std::cout << "Finished validating edge weights." << std::endl;


    std::cout << "Validating results..." << std::endl;
    bool validationFailed = false;
    double avgNodesInPath = 0;
    std::vector<NodeID> pathTree(graph.getNumNodes());
    for (auto query: queries) {
        auto source = query.source;
        std::vector<EdgeWeight> expectedDistances;
        for (auto target: query.targets) {
            auto path = DijkstraSearch().findShortestPath(graph, source, target, pathTree);
            avgNodesInPath += path.getNumLinks();
            expectedDistances.push_back(path.getLength());
        }

        std::vector<EdgeWeight> distances(numTargets, 0);

        for (auto method: methods) {
            bool methodFailed = false;
            method->findDistances(graph, query, distances);
            for (unsigned i = 0; i < numTargets; ++i) {
                if (expectedDistances[i] != distances[i]) {
                    methodFailed = true;
                }
            }
            if (methodFailed) {
                std::cout << method->getName() << " failed" << std::endl;
                validationFailed = true;
            }
        }
    }
    avgNodesInPath /= queries.size();

    std::cout << "Average nodes in path: " << avgNodesInPath << std::endl;

    if (!validationFailed) {
        std::cout << "Validation passed!" << std::endl;
    }
}

void DistanceExperimentCommand::runMethod(DistanceMethod *method)
{
    std::cout << "*** Measuring " << method->getName() << "... ***" << std::endl;
    method->printInfo();
    std::vector<EdgeWeight> distances(numTargets, 0);
    double cumulativeTime = 0;
    StopWatch sw;
    sw.start();
    for (unsigned i = 0; true; ++i) {
        if (isPowerOf(i, 2) || queries.size() == i) {
            std::cout << "    " << i << ", " << cumulativeTime << std::endl;
//                method->printStatistics();
            if (i >= queries.size()) {
                break;
            }
        }
        sw.reset();
        sw.start();

        method->findDistances(graph, queries[i], distances);

        sw.stop();
        cumulativeTime += sw.getTimeMs();
    }
    method->printStatistics();
}

void DistanceExperimentCommand::runMethod(DistanceMethod *method, int iter, ResultsType resultsType)
{
    std::cout << "*** Measuring " << method->getName() << "... ***" << std::endl;
    method->printInfo();
    std::vector<EdgeWeight> distances(numTargets, 0);
    double cumulativeTime = 0;
    StopWatch sw;
    sw.start();
    for (unsigned i = 0; true; ++i) {
        if (isPowerOf(i, 2) || queries.size() == i) {
            std::cout << "    " << i << ", " << cumulativeTime << std::endl;

            if (resultsType == ResultsType::POWER_OF_2 ||
                (resultsType == ResultsType::LAST && queries.size() == i)) {
                results[iter].push_back(cumulativeTime);
            }

//                method->printStatistics();
            if (i >= queries.size()) {
                break;
            }
        }
        sw.reset();
        sw.start();

        method->findDistances(graph, queries[i], distances);

        sw.stop();
        cumulativeTime += sw.getTimeMs();
    }
    method->printStatistics();
}


//void DistanceExperimentCommand::runMultiTargetALT()
//{
//    alt.edgesAccessedCount = 0;
//
//        alt.findShortestPathDistances(graph, query.source, query.targets);
//    }
////    std::cout << "Edges accessed: " << alt.edgesAccessedCount << std::endl;
//}

void DistanceExperimentCommand::clearMethods()
{
    for (auto method: methods) {
        delete method;
    }
    methods.clear();
}

DistanceExperimentCommand::~DistanceExperimentCommand()
{
    for (auto method: methods) {
        delete method;
    }
}

std::string DistanceExperimentCommand::getResultsPath()
{
    std::string workloadPrefix;
    switch (workloadType) {
        case WorkloadType::RANDOM:
            workloadPrefix = "";
            break;
        case WorkloadType::CLUSTER:
            workloadPrefix = "clustered_";
            break;
        default:
            std::cout << "Unknown workload type!" << std::endl;
            exit(1);
            break;
    }
    return resultsPath + "/" + workloadPrefix + network + "_results";
}



