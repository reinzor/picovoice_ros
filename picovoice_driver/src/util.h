#pragma once

#include <ostream>
#include <sstream>
#include <string>
#include <vector>

namespace picovoice_driver
{
struct KeyValue
{
  KeyValue(const std::string& key, const std::string& value);

  std::string key_;
  std::string value_;
};
std::ostream& operator<<(std::ostream& os, const KeyValue& kv);

template <typename T>
std::string toString(const T& v)
{
  std::stringstream ss;
  ss << v;
  return ss.str();
}

template <typename T>
std::string toString(const std::vector<T>& v)
{
  std::stringstream ss;
  ss << "{";
  for (size_t i = 0; i < v.size(); ++i)
  {
    if (i != 0)
    {
      ss << ", ";
    }
    ss << v[i];
  }
  ss << "}";
  return ss.str();
}
}  // namespace picovoice_driver
