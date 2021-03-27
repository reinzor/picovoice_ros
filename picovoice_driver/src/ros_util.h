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

#include <ros/package.h>
#include <string>

namespace picovoice_driver
{
//!
//! \brief defaultResourcePath Get the default resource path
//! \return Default resource path
//!
std::string defaultResourceUrl();

//!
//! \brief defaultRecordDirectory Get the default record dir
//! \return Default record dir
//!
std::string defaultRecordDirectory();

//!
//! \brief pathFromUrl Get a file path from an url
//!
//! URL options:
//!   - package://PACKAGE_NAME/relative/path/to/file.extension (relative to package)
//!   - file://absolute/path/to/file.extension (absolute file path)
//!   - file[.extension] (relative to specified directory if directory and extension specified)
//!
//! \param url Url
//! \param extension Extension
//! \param directory Optional directory
//! \return Path to file
//!
std::string pathFromUrl(const std::string& url, const std::string& extension = "", const std::string& directory = "");
}  // namespace picovoice_driver
