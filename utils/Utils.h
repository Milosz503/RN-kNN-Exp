//
// Created by milos on 28/05/2024.
//

#ifndef ND_KNN_UTILS_H
#define ND_KNN_UTILS_H

#include <cmath>
#include <vector>
#include <fstream>
#include "../command/shortest_distance/DistanceMethod.h"

bool isPowerOf(int number, int base) {
    if (number <= 0 || base <= 1) {
        return false;
    }

    double logResult = std::log(number) / std::log(base);
    return std::floor(logResult) == logResult;
}



void write_to_csv(const std::vector<std::vector<double>>& results, std::vector<std::string> methods, std::string path, int numOfRepeats)
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

void write_to_csv(const std::vector<std::vector<double>>& results, std::vector<DistanceMethod*> methods, std::string path, int numOfRepeats) {
    std::vector<std::string> methodNames;

    for (auto method : methods) {
        methodNames.push_back(method->getInfo());
    }

    write_to_csv(results, methodNames, path, numOfRepeats);
}


double func(unsigned queries) {
    if (queries < 2000)
        return queries * (-0.000063) + 0.3;
    return queries * -0.000008 + 0.19;
}

double exp_func(unsigned queries) {
    return 0.15 + 0.25 * std::exp(queries * -0.00157);
}

#endif //ND_KNN_UTILS_H
