//
// Created by milos on 08/04/2024.
//

#ifndef ND_KNN_ALTTEST_H
#define ND_KNN_ALTTEST_H

#include "UnitTest.h"
#include "../../processing/ier/ObjectList.h"

class ALTTest : public UnitTest {
public:
    void run() override {
        ObjectList objectList;
        std::vector<ObjectListElement> distances = {
                // landmark #1
                {21, 10},
                {12, 20},
                {33, 30},
                {74, 40},
                {35, 50},
                // landmark #2
                {74, 10},
                {21, 20},
                {12, 30},
                {35, 40},
                {33, 50},
        };
        objectList.setDistances(distances, 5);

        runTest("Should find closest object", [this, &objectList]() {
            int index;
            auto closestObject = objectList
                    .getClosestObject(0, 26, index);
            shouldBeEqual(closestObject.first, 33u);
            shouldBeEqual(closestObject.second, 30u);
            shouldBeEqual(index, 2);
        });

        runTest("Should find closest object", [this, &objectList]() {
            int index;
            auto closestObject = objectList
                    .getClosestObject(0, 24, index);
            shouldBeEqual(closestObject.first, 12u);
            shouldBeEqual(closestObject.second, 20u);
            shouldBeEqual(index, 1);
        });

        runTest("Should find next closest object", [this, &objectList]() {
            int leftIndex = 2;
            int rightIndex = 2;
            auto closestObject = objectList
                    .getNextClosestObject(0, 26, leftIndex, rightIndex);
            shouldNotBeNull(closestObject);
            shouldBeEqual(closestObject->first, 12u);
            shouldBeEqual(leftIndex, 1);
            shouldBeEqual(rightIndex, 2);
        });

        runTest("Should find next closest object", [this, &objectList]() {
            int leftIndex = 1;
            int rightIndex = 2;
            auto closestObject = objectList
                    .getNextClosestObject(0, 26, leftIndex, rightIndex);
            shouldNotBeNull(closestObject);
            shouldBeEqual(closestObject->first, 74u);
            shouldBeEqual(leftIndex, 1);
            shouldBeEqual(rightIndex, 3);
        });

        runTest("Should find right closest object in #2 landmark", [this, &objectList]() {
            int leftIndex = 5;
            int rightIndex = 8;
            auto closestObject = objectList
                    .getNextClosestObject(1, 26, leftIndex, rightIndex);
            shouldNotBeNull(closestObject);
            shouldBeEqual(closestObject->first, 33u);
            shouldBeEqual(leftIndex, 5);
            shouldBeEqual(rightIndex, 9);
        });

        runTest("Should find left closest object in #2 landmark", [this, &objectList]() {
            int leftIndex = 6;
            int rightIndex = 9;
            auto closestObject = objectList
                    .getNextClosestObject(1, 26, leftIndex, rightIndex);
            shouldNotBeNull(closestObject);
            shouldBeEqual(closestObject->first, 74u);
            shouldBeEqual(leftIndex, 5);
            shouldBeEqual(rightIndex, 9);
        });

        runTest("Should not find next closest object", [this, &objectList]() {
            int leftIndex = 0;
            int rightIndex = 4;
            auto closestObject = objectList
                    .getNextClosestObject(0, 26, leftIndex, rightIndex);
            shouldBeNull(closestObject);
        });

    }

};

#endif //ND_KNN_ALTTEST_H
