//
// Created by milos on 14/02/2024.
//

#include "DistanceMatrix.h"

void DistanceMatrix::init(unsigned int targetsLength, unsigned int sourcesLength) {
    rowLength = targetsLength;
    distanceMatrix.reserve(sourcesLength*rowLength);
}

size_t DistanceMatrix::capacity() {
    return distanceMatrix.capacity();
}
