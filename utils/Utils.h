//
// Created by milos on 28/05/2024.
//

#ifndef ND_KNN_UTILS_H
#define ND_KNN_UTILS_H

#include <cmath>
#include <vector>
#include <fstream>

bool isPowerOf(int number, int base) {
    if (number <= 0 || base <= 1) {
        return false;
    }

    double logResult = std::log(number) / std::log(base);
    return std::floor(logResult) == logResult;
}

void write_to_csv(const std::vector<std::vector<double>>& results, std::vector<DistanceMethod*> methods, std::string network, int numOfRepeats) {
    std::ofstream file(network + "_output.csv");
    std::ofstream file_deviation(network + "_output_dev.csv");

    for (int i = 0; i < results.size(); ++i) {
        file << methods[i]->getInfo() << ",";
    }
    file << std::endl;

    int numMeasurements = results[0].size() / numOfRepeats;
    for (int j = 0; j < numMeasurements; j++) {
        for (auto i = 0; i < results.size(); i++) {
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
            file << avg << ",";
            file_deviation << stdder << ",";
        }
        file << std::endl;
        file_deviation << std::endl;
    }

    file.close();
    file_deviation.close();
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
