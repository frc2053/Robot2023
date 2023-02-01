
#pragma once

#include <map>
#include <utility>
#include <frc/geometry/Translation2d.h>

namespace str {
template <typename Value>
class interpolating_map_xy {
 public:
  void insert(const frc::Translation2d& key, const Value& value) {
    m_container.insert(std::make_pair(key, value));
  }

  void insert(frc::Translation2d&& key, Value&& value) {
    m_container.insert(std::make_pair(key, value));
  }

  Value operator[](const frc::Translation2d& key) const {
    using const_iterator = typename std::map<frc::Translation2d, Value>::const_iterator;

    // Get iterator to upper bound key-value pair for the given key
    const_iterator upper = m_container.upper_bound(key);

    // If key > largest key in table, return value for largest table key
    if (upper == m_container.end()) {
      return (--upper)->second;
    }

    // If key <= smallest key in table, return value for smallest table key
    if (upper == m_container.begin()) {
      return upper->second;
    }

    // Get iterator to lower bound key-value pair
    const_iterator lower = upper;
    --lower;

    // Perform linear interpolation between lower and upper bound
    const units::meter_t deltaX = (key.X() - lower->first.X()) / (upper->first.X() - lower->first.X());
    const units::meter_t deltaY = (key.Y() - lower->first.Y()) / (upper->first.Y() - lower->first.Y());
    Value resultX = deltaX.value() * upper->second + (1.0 - deltaX.value()) * lower->second;
    return resultX;
  }

  /**
   * Clears the contents.
   */
  void clear() { m_container.clear(); }

 private:
  std::map<frc::Translation2d, Value> m_container;
};

}  // namespace str