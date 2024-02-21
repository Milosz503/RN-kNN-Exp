//
// Created by milos on 21/02/2024.
//

#ifndef ND_KNN_LOGGER_H
#define ND_KNN_LOGGER_H

#include <string>
#include <iostream>

//#define LOGGING

class Logger {
public:
    static void log() {
        std::cout << std::endl;
    }

    template<typename T, typename... Args>
    static void log(const T& firstLog, const Args&... logs) {
#ifdef LOGGING
        std::cout << firstLog;
        log(logs...);
#endif
    }
};

#endif //ND_KNN_LOGGER_H
