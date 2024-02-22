//
// Created by milos on 14/02/2024.
//

#ifndef ND_KNN_DISTANCEMATRIX_H
#define ND_KNN_DISTANCEMATRIX_H

#include "../Graph.h"
#include "../Gtree.h"
#include "../../queue/BinaryMinHeap.h"
#include "../../utility/METISWrapper.h"
#include "../../utility/Statistics.h"

#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/unordered_set.hpp>
#include <boost/serialization/unordered_map.hpp>
#include <iostream>


class DistanceMatrix {

public:

    void init(unsigned targetsLength, unsigned sourcesLength);

    void push_back(EdgeWeight distance);

    void pushBackNotAssigned();

    EdgeWeight atIndex(unsigned i);

    EdgeWeight get(unsigned row, unsigned column);

    bool isAssigned(unsigned row, unsigned column);

    bool isAssigned(unsigned int index);

    void set(unsigned row, unsigned column, EdgeWeight weight);

    size_t size();

    size_t capacity();

private:
    friend class boost::serialization::access;

    std::vector<long> distanceMatrix;
    unsigned rowLength;

    // Boost Serialization
    template<class Archive>
    void serialize(Archive & ar, const unsigned int version)
    {
        ar & this->distanceMatrix;
        ar & this->rowLength;
    }
};

inline void DistanceMatrix::push_back(EdgeWeight distance) {
    distanceMatrix.push_back(distance);
}

inline void DistanceMatrix::pushBackNotAssigned() {
    distanceMatrix.push_back(-1);
}

inline EdgeWeight DistanceMatrix::atIndex(unsigned i) {
    auto dist = distanceMatrix[i];
    assert(dist >= 0);
    return dist;
}

inline EdgeWeight DistanceMatrix::get(unsigned int row, unsigned int column) {
    auto dist = distanceMatrix[row*rowLength + column];
    assert(dist >= 0);
    return dist;
}

inline bool DistanceMatrix::isAssigned(unsigned int row, unsigned int column) {
    auto dist = distanceMatrix[row*rowLength + column];
    return dist >= 0;
}

inline bool DistanceMatrix::isAssigned(unsigned int index) {
    return distanceMatrix[index] >= 0;
}

inline void DistanceMatrix::set(unsigned int row, unsigned int column, EdgeWeight weight) {
    distanceMatrix[row*rowLength + column] = weight;
}

inline size_t DistanceMatrix::size() {
    return distanceMatrix.size();
}




#endif //ND_KNN_DISTANCEMATRIX_H
