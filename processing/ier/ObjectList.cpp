//
// Created by milos on 08/04/2024.
//

#include "ObjectList.h"

void ObjectList::setDistances(std::vector<ObjectListElement> &distances, int objectsNumber)
{
    objectLists = std::move(distances);
    this->objectsNumber = objectsNumber;
}


ObjectListElement ObjectList::getClosestObject(int landmark, EdgeWeight distanceFromLandmark, int &index)
{
    int startIndex = getStartIndex(landmark);
    int endIndex = getEndIndex(landmark);

    while(endIndex - startIndex > 1) {
        int middleIndex = (startIndex + endIndex) / 2;
        if(objectLists[middleIndex].second < distanceFromLandmark) {
            startIndex = middleIndex;
        }
        else {
            endIndex = middleIndex;
        }
    }

    if(startIndex == endIndex) {
        index = startIndex;
        return objectLists[startIndex];
    }

    int leftDistance = objectLists[startIndex].second - distanceFromLandmark;
    leftDistance = std::abs(leftDistance);
    int rightDistance = objectLists[endIndex].second - distanceFromLandmark;
    rightDistance = std::abs(rightDistance);

    if(leftDistance < rightDistance) {
        index = startIndex;
        return objectLists[startIndex];
    }

    index = endIndex;
    return objectLists[endIndex];
}

ObjectListElement *ObjectList::getNextClosestObject(int landmark, EdgeWeight distanceFromLandmark, int &leftIndex, int &rightIndex)
{
    int startIndex = getStartIndex(landmark);
    int endIndex = getEndIndex(landmark);

    int leftCandidate = leftIndex - 1;
    int rightCandidate = rightIndex + 1;

    if(leftCandidate < startIndex && rightCandidate > endIndex) {
        return nullptr;
    }
    if(leftCandidate < startIndex) {
        rightIndex = rightCandidate;
        return &objectLists[rightIndex];
    }
    if(rightCandidate > endIndex) {
        leftIndex = leftCandidate;
        return &objectLists[leftIndex];
    }

    int leftDistance = objectLists[leftCandidate].second - distanceFromLandmark;
    leftDistance = std::abs(leftDistance);
    int rightDistance = objectLists[rightCandidate].second - distanceFromLandmark;
    rightDistance = std::abs(rightDistance);

    if(leftDistance < rightDistance) {
        leftIndex = leftCandidate;
        return &objectLists[leftIndex];
    }

    rightIndex = rightCandidate;
    return &objectLists[rightIndex];
}

int ObjectList::getStartIndex(int landmark)
{
    return landmark * objectsNumber;
}

int ObjectList::getEndIndex(int landmark)
{
    return (landmark + 1) * objectsNumber-1;
}


