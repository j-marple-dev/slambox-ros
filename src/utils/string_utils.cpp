/// @file
/// @author Haneol Kim <hekim@jmarple.ai>
/// @copyright 2023 J.Marple
/// @brief String utilities.

#include "utils/string_utils.hpp"

namespace sbox::string_utils {
std::vector<std::string> split(std::string input, char delimiter) {
  std::vector<std::string> result;
  std::stringstream ss(input);
  std::string temp;

  while (getline(ss, temp, delimiter)) {
    result.push_back(temp);
  }

  return result;
}
}  // namespace sbox::string_utils
