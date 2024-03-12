//
// Created by milos on 12/03/2024.
//

#include "TestsCommand.h"

void TestsCommand::runAllTests()
{
    //TODO: Add tests here

    runTest("Test 1", [this]() {
        int a = 1;
        int b = 1;
        shouldBeEqual(a, b);
    });

    runTest("Test 2", [this]() {
        int a = 1;
        int b = 2;
        shouldBeEqual(a, b);
    });
}


void TestsCommand::execute(int argc, char **argv)
{
    std::cout << "Running tests..." << std::endl;

    runAllTests();

    std::cout << "\n\n";

    if (failedTests > 0) {
        printHighlightedError(std::to_string(failedTests) + " TESTS FAILED!");
    } else {
        printSuccess("All tests passed!");
    }

}

void TestsCommand::showCommandUsage(std::string programName)
{
    std::cerr << "Usage: " << programName << " -c " + constants::TESTS_CMD + " all\n\n";
}

void TestsCommand::runTest(std::string test, std::function<void()> testFunc)
{
    currentTest = test;
    testFunc();
    printSuccess("Test passed: " + currentTest);
    currentTest = "undefined";
}
