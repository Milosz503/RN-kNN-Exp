//
// Created by milos on 12/03/2024.
//

#include "TestsCommand.h"
#include "UnitTest.h"
#include "ALTTest.h"
#include "DijkstraTest.h"


void TestsCommand::execute(int argc, char **argv)
{
    UnitTest::runTests(
            ALTTest(),
            DijkstraTest()
    );
}

void TestsCommand::showCommandUsage(std::string programName)
{
    std::cerr << "Usage: " << programName << " -c " + constants::TESTS_CMD + " all\n\n";
}
