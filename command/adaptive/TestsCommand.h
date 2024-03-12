//
// Created by milos on 12/03/2024.
//

#ifndef ND_KNN_TESTSCOMMAND_H
#define ND_KNN_TESTSCOMMAND_H


#include <string>
#include <functional>
#include "../Command.h"
#include "../../common.h"

class TestsCommand : public Command {

public:
    void execute(int argc, char *argv[]) override;

    void showCommandUsage(std::string programName) override;


private:
    std::string currentTest;
    int failedTests = 0;

    void runAllTests();

    void runTest(std::string test, std::function<void()> testFunc);

    template<typename T>
    void shouldBeEqual(const T &current, const T &expected)
    {
        if (current != expected) {
            failedTests++;
            printError("Test failed: " + currentTest);
            std::cout << "   Expected: " << std::to_string(expected) << ", but got: " << std::to_string(current)
                      << std::endl;
        }
    }

    static void printError(const std::string &message)
    {
        std::cerr << "\033[21;31m" << message << "\033[0m" << std::endl;
    }

    static void printHighlightedError(const std::string &message)
    {
        std::cerr << "\033[1;31m" << message << "\033[0m" << std::endl;
    }

    static void printSuccess(const std::string &message)
    {
        std::cout << "\033[21;32m" << message << "\033[0m" << std::endl;
    }
};


#endif //ND_KNN_TESTSCOMMAND_H
