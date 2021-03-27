/*
 * Copyright 2021, Rein Appeldoorn
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

#pragma once

#include <map>
#include <sstream>
#include <string>
#include <vector>

namespace picovoice_driver
{
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

template <typename T1, typename T2>
std::string toString(const std::map<T1, T2>& v)
{
  std::stringstream ss;
  ss << "{";
  bool first = true;
  for (const auto& kv : v)
  {
    if (first)
    {
      first = false;
    }
    else
    {
      ss << ", ";
    }
    ss << kv.first << "=" << kv.second;
  }
  ss << "}";
  return ss.str();
}
}  // namespace picovoice_driver
