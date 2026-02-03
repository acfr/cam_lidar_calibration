#ifndef FLANN_FIX_H
#define FLANN_FIX_H

// Workaround for FLANN serialization issues
// Add serialize methods to std::unordered_map and boost::dynamic_bitset
namespace std {
  template<typename K, typename V>
  struct unordered_map;
}

namespace boost {
  template<typename Block, typename Allocator>
  class dynamic_bitset;
}

// Provide minimal serialization interface
namespace std {
  template<typename K, typename V>
  template<typename Archive>
  void unordered_map<K,V>::serialize(Archive& ar) {
    // Dummy implementation to satisfy FLANN template instantiation
    (void)ar;
  }
}

namespace boost {
  template<typename Block, typename Allocator>
  template<typename Archive>
  void dynamic_bitset<Block,Allocator>::serialize(Archive& ar) {
    // Dummy implementation to satisfy FLANN template instantiation
    (void)ar;
  }
}

#endif // FLANN_FIX_H
