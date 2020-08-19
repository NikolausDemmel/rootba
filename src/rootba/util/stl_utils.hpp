/**
BSD 3-Clause License

This file is part of the RootBA project.
https://github.com/NikolausDemmel/rootba

Copyright (c) 2021, Nikolaus Demmel.
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/
#pragma once

#include <algorithm>
#include <map>
#include <set>
#include <string_view>
#include <unordered_set>
#include <vector>

namespace rootba {

/// return set of keys for given map
template <class M>
auto get_keys(const M& map) {
  std::set<typename M::key_type> keys;
  std::transform(map.begin(), map.end(), std::inserter(keys, keys.end()),
                 [](const auto& pair) { return pair.first; });
  return keys;
}

/// return vector of keys for given map
template <class M>
auto get_key_vector(const M& map) {
  std::vector<typename M::key_type> keys;
  std::transform(map.begin(), map.end(), std::back_inserter(keys),
                 [](const auto& pair) { return pair.first; });
  return keys;
}

/// convert range to set
template <typename Seq>
auto to_set(Seq&& seq) {
  return std::set<std::decay_t<decltype(*std::begin(std::declval<Seq&>()))>>{
      std::begin(seq), std::end(seq)};
}

/// convert range to vector
template <typename Seq>
auto to_vector(Seq&& seq) {
  return std::vector<std::decay_t<decltype(*std::begin(std::declval<Seq&>()))>>{
      std::begin(seq), std::end(seq)};
}

/// check if set a is subset of set b
template <typename T, typename C, typename A>
bool is_subset_of(const std::set<T, C, A>& a, const std::set<T, C, A>& b) {
  return std::includes(b.begin(), b.end(), a.begin(), a.end());
}

/// set difference for std::set; computes a - b
template <typename T, typename C, typename A>
std::set<T, C, A> set_difference(const std::set<T, C, A>& a,
                                 const std::set<T, C, A>& b) {
  std::set<T, C, A> ret;
  std::set_difference(a.begin(), a.end(), b.begin(), b.end(),
                      std::inserter(ret, ret.end()));
  return ret;
}

/// set difference for std::set; computes a - b
template <typename T, typename H, typename P, typename A>
std::unordered_set<T, H, P, A> set_difference(
    const std::unordered_set<T, H, P, A>& a,
    const std::unordered_set<T, H, P, A>& b) {
  std::unordered_set<T, H, P, A> ret;
  for (const T& x : a) {
    if (!b.count(x)) {
      ret.insert(x);
    }
  }
  return ret;
}

/// set union for std::set; computes a + b
template <typename T, typename C, typename A>
std::set<T, C, A> set_union(const std::set<T, C, A>& a,
                            const std::set<T, C, A>& b) {
  std::set<T, C, A> result = a;
  result.insert(b.begin(), b.end());
  return result;
}

/// set union for std::set; computes a + b
template <typename T, typename H, typename P, typename A>
std::unordered_set<T, H, P, A> set_union(
    const std::unordered_set<T, H, P, A>& a,
    const std::unordered_set<T, H, P, A>& b) {
  std::unordered_set<T, H, P, A> result = a;
  result.insert(b.begin(), b.end());
  return result;
}

/// set intersections for std::set; computes a \intersection b
template <typename T, typename C, typename A>
std::set<T, C, A> set_intersection(const std::set<T, C, A>& a,
                                   const std::set<T, C, A>& b) {
  std::set<T, C, A> ret;
  std::set_intersection(a.begin(), a.end(), b.begin(), b.end(),
                        std::inserter(ret, ret.end()));
  return ret;
}

/// for set types, update one set with copied entries from a second
template <class S>
void update_set(S& a, const S& b) {
  for (const auto& v : b) {
    a.insert(v);
  }
}

/// string ends_with
inline bool ends_with(const std::string_view str,
                      const std::string_view suffix) {
  if (str.length() < suffix.length()) {
    return false;
  }
  return (0 ==
          str.compare(str.length() - suffix.length(), suffix.length(), suffix));
}

/// guarantee to clear memory of a vector
/// Note: vector.clear() does in general not clear the memory
template <typename T, typename A>
inline void vector_clear_memory(std::vector<T, A>& vector) {
  std::vector<T, A>().swap(vector);
}

/// erase all elements matching a predicate from a vector
template <typename T, typename A, typename P>
inline void erase_if(std::vector<T, A>& vector, const P& predicate) {
  vector.erase(std::remove_if(vector.begin(), vector.end(), predicate),
               vector.end());
}

/// erase all elements matching a predicate from map or set
template <typename M, typename P>
inline std::void_t<typename M::key_type> erase_if(M& map, const P& predicate) {
  for (auto it = begin(map), end_it(end(map)); it != end_it;) {
    if (predicate(*it)) {
      it = map.erase(it);
    } else {
      ++it;
    }
  }
}

/// unsafe_erase all elements matching a predicate from map or set
template <typename M, typename P>
inline std::void_t<typename M::key_type> unsafe_erase_if(M& map,
                                                         const P& predicate) {
  for (auto it = begin(map), end_it(end(map)); it != end_it;) {
    if (predicate(*it)) {
      it = map.unsafe_erase(it);
    } else {
      ++it;
    }
  }
}

/// for map types with pointer value type, return an entry or if not present
/// return pointer constructed from nullptr
template <class M>
const typename M::mapped_type& get_or_null(const M& map,
                                           const typename M::key_type& key) {
  static const typename M::mapped_type null{nullptr};
  auto it = map.find(key);
  if (it == map.end()) {
    return null;
  } else {
    return it->second;
  }
}

/// for map types, return an entry or if not present return the provided default
/// value
template <class M>
const typename M::mapped_type& get_or_default(
    const M& map, const typename M::key_type& key,
    const typename M::mapped_type& default_value) {
  auto it = map.find(key);
  if (it == map.end()) {
    return default_value;
  } else {
    return it->second;
  }
}

/// for map types, update one map with copied entries from a second
template <class M>
void update_map(M& a, const M& b) {
  for (const auto& [k, v] : b) {
    a[k] = v;
  }
}

/// sort a vector
template <typename T, typename A>
inline void sort(std::vector<T, A>& v) {
  std::sort(std::begin(v), std::end(v));
}

/// apply functor to all elements of a vector
template <typename T, typename A, typename F>
inline void for_each(std::vector<T, A>& v, F f) {
  for (auto& e : v) {
    f(e);
  }
}

}  // namespace rootba
