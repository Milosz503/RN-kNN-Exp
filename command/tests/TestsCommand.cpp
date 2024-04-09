//
// Created by milos on 12/03/2024.
//

#include "TestsCommand.h"
#include "UnitTest.h"
#include "ALTTest.h"


void TestsCommand::execute(int argc, char **argv)
{
    UnitTest::runTests(
            ALTTest()
    );
}

void TestsCommand::showCommandUsage(std::string programName)
{
    std::cerr << "Usage: " << programName << " -c " + constants::TESTS_CMD + " all\n\n";
}
