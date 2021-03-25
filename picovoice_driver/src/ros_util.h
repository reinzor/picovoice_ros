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
