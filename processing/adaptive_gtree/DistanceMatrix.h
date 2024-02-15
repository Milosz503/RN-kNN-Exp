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


class DistanceMatrix {

public:

    void init(unsigned targetsLength, unsigned sourcesLength);

    void push_back(EdgeWeight distance);

    EdgeWeight atIndex(unsigned i);

    EdgeWeight get(unsigned row, unsigned column);

    size_t size();

    size_t capacity();

private:
    friend class boost::serialization::access;

    std::vector<EdgeWeight> distanceMatrix;
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

inline EdgeWeight DistanceMatrix::atIndex(unsigned i) {
    return distanceMatrix[i];
}

inline EdgeWeight DistanceMatrix::get(unsigned int row, unsigned int column) {
    return distanceMatrix[row*rowLength + column];
}


inline size_t DistanceMatrix::size() {
    return distanceMatrix.size();
}

#endif //ND_KNN_DISTANCEMATRIX_H
