/// @file
/// @author Haneol Kim <hekim@jmarple.ai>
/// @copyright 2023 J.Marple
/// @brief Test the utility functions for string

#include <glog/logging.h>
#include <gtest/gtest.h>

#include <random>
#include <string>
#include <vector>

#include "utils/string_utils.hpp"

TEST(StringUtils, SplitString) {
  std::vector<std::string> expected_result;
  std::vector<std::string> result;

  std::string test_string("TEST,STRING,ABC,");
  expected_result.push_back("TEST");
  expected_result.push_back("STRING");
  expected_result.push_back("ABC");

  result = sbox::string_utils::split(test_string, ',');

  EXPECT_EQ(expected_result.size(), result.size());

  for (int i; i < result.size(); i++) {
    EXPECT_EQ(result.at(i), expected_result.at(i));
  }
}

TEST(StringUtils, SplitStringRandom) {
  std::random_device rd;
  std::mt19937 gen(rd());

  std::uniform_int_distribution<uint16_t> dis_str_length(1, 10000);
  std::uniform_int_distribution<uint8_t> dis_char(33, 126);

  for (int i = 0; i < 1000; i++) {
    uint8_t delimiter = dis_char(gen);

    // Generate random string except delimiter
    std::string test_string;
    test_string.resize(dis_str_length(gen));
    for (int j = 0; j < test_string.size(); j++) {
      test_string.at(j) = dis_char(gen);
      while (test_string.at(j) == delimiter) {
        test_string.at(j) = dis_char(gen);
      }
    }

    // Generate random delimiter indices
    std::uniform_int_distribution<uint8_t> dis_delimiter_length(
        0, fmin(255, test_string.size() - 1));
    uint8_t delimiter_length = dis_delimiter_length(gen);
    std::vector<uint16_t> delimiter_indices(delimiter_length);
    for (int j = 0; j < delimiter_length; j++) {
      std::uniform_int_distribution<uint16_t> dis_delimiter_index(
          0, test_string.size() - 1);
      delimiter_indices.at(j) = dis_delimiter_index(gen);
    }
    std::sort(delimiter_indices.begin(), delimiter_indices.end());

    // Remove duplicated delimiter index
    delimiter_indices.erase(
        std::unique(delimiter_indices.begin(), delimiter_indices.end()),
        delimiter_indices.end());
    delimiter_length = delimiter_indices.size();

    std::vector<std::string> answer_result(delimiter_length);

    // Insert delimiter and stack answer split string
    for (int j = 0; j < delimiter_length; j++) {
      test_string.at(delimiter_indices.at(j)) = delimiter;

      int s_idx = j == 0 ? 0 : delimiter_indices.at(j - 1) + 1;
      int n_char = delimiter_indices.at(j) -
                   (j == 0 ? 0 : delimiter_indices.at(j - 1) + 1);

      answer_result.at(j) = test_string.substr(s_idx, n_char);
    }
    // Handle edge case
    if (delimiter_length == 0) {
      answer_result.push_back(test_string);
    } else if (delimiter_indices.back() < test_string.size() - 1) {
      answer_result.push_back(test_string.substr(delimiter_indices.back() + 1));
    }

    // Test
    std::vector<std::string> result =
        sbox::string_utils::split(test_string, delimiter);

    EXPECT_EQ(answer_result.size(), result.size());
    for (int j = 0; j < result.size(); j++) {
      EXPECT_EQ(answer_result.at(j), result.at(j));
    }
  }
}
