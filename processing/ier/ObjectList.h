//
// Created by milos on 08/04/2024.
//

#ifndef ND_KNN_OBJECTLIST_H
#define ND_KNN_OBJECTLIST_H


#include <utility>
#include <vector>
#include "../../common.h"

typedef std::pair<NodeID, EdgeWeight> ObjectListElement;

class ObjectList {
public:
    void setDistances(std::vector<ObjectListElement>& distances, int objectsNumber);

    ObjectListElement getClosestObject(int landmark, EdgeWeight distanceFromLandmark, int& index);
    ObjectListElement* getNextClosestObject(int landmark, EdgeWeight distanceFromLandmark, int& leftIndex, int& rightIndex);

private:
    std::vector<ObjectListElement> objectLists;
    int objectsNumber;

    int getStartIndex(int landmark);
    int getEndIndex(int landmark);
};


#endif //ND_KNN_OBJECTLIST_H
