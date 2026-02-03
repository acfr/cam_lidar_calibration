/*
 * FLANN Serialization Fix for libflann 1.9.2
 * 
 * This header provides missing serialize() methods that FLANN expects
 * but are not present in modern C++ standard library containers.
 * This is a workaround for a known bug in FLANN 1.9.2.
 */

#ifndef FLANN_SERIALIZATION_FIX_H
#define FLANN_SERIALIZATION_FIX_H

#include <unordered_map>
#include <boost/dynamic_bitset.hpp>

// Add serialize method to std::unordered_map for FLANN compatibility
namespace std {
template<typename K, typename V>
class unordered_map;
}

namespace boost {
template<typename Block, typename Allocator>
class dynamic_bitset;
}

// Forward declare Archive types
namespace flann {
namespace serialization {
class SaveArchive;
class LoadArchive;
}
}

// Provide serialize method implementations
namespace std {
template<typename K, typename V>
void serialize(flann::serialization::SaveArchive& ar, std::unordered_map<K,V>& obj);

template<typename K, typename V>
void serialize(flann::serialization::LoadArchive& ar, std::unordered_map<K,V>& obj);
}

namespace boost {
template<typename Block, typename Allocator>
void serialize(flann::serialization::SaveArchive& ar, dynamic_bitset<Block,Allocator>& obj);

template<typename Block, typename Allocator>
void serialize(flann::serialization::LoadArchive& ar, dynamic_bitset<Block,Allocator>& obj);
}

#endif // FLANN_SERIALIZATION_FIX_H
