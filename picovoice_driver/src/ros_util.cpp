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

#include <boost/algorithm/string/predicate.hpp>
#include <boost/algorithm/string/regex.hpp>
#include <boost/filesystem.hpp>

#include "./ros_util.h"
#include "./util.h"

namespace picovoice_driver
{
std::string defaultResourceUrl()
{
  return "package://picovoice_driver/resources";
}

std::string defaultRecordDirectory(const std::string& name)
{
  return std::string("/tmp/picovoice_driver/" + name);
}

void validatePathExistence(const std::string& path, const std::string& extension = "")
{
  if (!boost::filesystem::exists(path))
  {
    throw std::runtime_error("Path '" + path + "' does not exist");
  }
  if (!extension.empty() && !boost::algorithm::ends_with(path, extension))
  {
    throw std::runtime_error("Path '" + path + "' does not match extension '" + extension + "'");
  }
}

std::string pathFromUrl(const std::string& url, const std::string& extension, const std::string& directory)
{
  try
  {
    if (url.find("://") == std::string::npos && !directory.empty())
    {
      validatePathExistence(directory);

      if (boost::filesystem::exists(directory + "/" + url) && boost::algorithm::ends_with(url, extension))
      {
        return directory + "/" + url;
      }
      if (boost::filesystem::exists(directory + "/" + url + extension))
      {
        return directory + "/" + url + extension;
      }

      std::vector<std::string> candidates;
      for (const auto& entry : boost::make_iterator_range(boost::filesystem::directory_iterator(directory)))
      {
        if (entry.path().extension().string() == extension)
        {
          candidates.push_back(entry.path().stem().string());
        }
      }
      throw std::runtime_error("Invalid relative file '" + url + "'. Available: " + toString(candidates));
    }

    std::vector<std::string> parts;
    boost::algorithm::split_regex(parts, url, boost::regex("://"));
    if (parts.size() != 2)
    {
      throw std::runtime_error("Invalid package or file url");
    }
    const std::string& url_type = parts.front();
    const std::string& url_arg = parts.back();

    if (url_type == "package")
    {
      size_t first_slash_pos = url_arg.find("/");
      std::string package = url_arg.substr(0, first_slash_pos);
      std::string path_relative_to_package = first_slash_pos < url_arg.size() ? url_arg.substr(first_slash_pos) : "";

      auto pkg_path = ros::package::getPath(package);
      if (pkg_path.empty())
      {
        throw std::runtime_error("Could not find package + '" + package + "'");
      }
      auto path = pkg_path + path_relative_to_package;
      validatePathExistence(path, extension);
      return path;
    }
    else if (url_type == "file")
    {
      validatePathExistence(url_arg, extension);
      return url_arg;
    }
    throw std::runtime_error("Invalid url type '" + url_type + "'");
  }
  catch (const std::exception& e)
  {
    std::string err = "Failed to get path from url '" + url + "': " + std::string(e.what()) +
                      " - The url should be of form {package://path_relative_to_package" + extension +
                      ", file:///absolute/path" + extension + "}";
    if (!directory.empty() && !extension.empty())
    {
      err += " or a filename[" + extension + "] relative to directory '" + directory + "'";
    }
    throw std::runtime_error(err);
  }
}
}  // namespace picovoice_driver
