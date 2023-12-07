/// @file
/// @author Haneol Kim <hekim@jmarple.ai>
/// @copyright 2023 J.Marple
/// @brief String utilities.

#ifndef SLAMBOX_ROS_INCLUDE_UTILS_STRING_UTILS_HPP_
#define SLAMBOX_ROS_INCLUDE_UTILS_STRING_UTILS_HPP_

#include <sstream>
#include <string>
#include <vector>

namespace sbox::string_utils {

/// @brief Method for spliting string
/// @param input string to split
/// @param delimiter a delimiter used for spliting the input string
/// @return a vector contains splited strings
std::vector<std::string> split(std::string input, char delimiter);
}  // namespace sbox::string_utils

#endif  // SLAMBOX_ROS_INCLUDE_UTILS_STRING_UTILS_HPP_
