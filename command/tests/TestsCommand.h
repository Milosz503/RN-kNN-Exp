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
};


#endif //ND_KNN_TESTSCOMMAND_H
