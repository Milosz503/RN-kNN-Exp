//
// Created by milos on 08/04/2024.
//

#ifndef ND_KNN_UNITTEST_H
#define ND_KNN_UNITTEST_H

#include <string>
#include <functional>
#include <iostream>
#include <vector>

class UnitTest {
public:
    template<typename... Args>
    static void runTests(Args... tests)
    {
        std::cout << "Running tests..." << std::endl;
        runAllTests(tests...);
        std::cout << "\n\n";
        printSummary();
    }

    static void printSummary()
    {
        if (failedTests > 0) {
            printHighlightedError(std::to_string(failedTests) + " TESTS FAILED!");
            printSuccess(std::to_string(testCount - failedTests) + " tests passed!");
        } else {
            printSuccess("All tests passed!");
        }
    }

    virtual void run() = 0;

protected:

    void runTest(std::string test, std::function<void()> testFunc)
    {
        currentTest = test;
        testCount++;
        try {
            testFunc();
            printSuccess("Test passed: " + currentTest);
        } catch (const std::exception &e) {
            failedTests++;
            printError("Test failed: " + currentTest);
            std::cout << "   " << std::string(e.what()) << std::endl;
        }
        currentTest = "undefined";
    }

    template<typename T>
    void shouldBeEqual(const T &current, const T &expected)
    {
        if (current != expected) {
            throw std::runtime_error("Expected: " + std::to_string(expected) + ", but got: " + std::to_string(current));
        }
    }

    template<typename T>
    void shouldNotBeNull(const T* current)
    {
        if (current == nullptr) {
            throw std::runtime_error("Expected not null, but got null");
        }
    }

    template<typename T>
    void shouldBeNull(const T* current)
    {
        if (current != nullptr) {
            throw std::runtime_error("Expected null, but got not null");
        }
    }

private:
    std::string currentTest = "undefined";
    static int testCount;
    static int failedTests;

    static void runAllTests() {}

    template<typename T, typename... Args>
    static void runAllTests(T& firstTest, Args&... tests) {
        firstTest.run();
        runAllTests(tests...);
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



#endif //ND_KNN_UNITTEST_H
