/**
 * @file helper_functions_test.cpp
 * @author Venkata Madhav
 * @brief
 * @version 0.1
 * @date 2025-12-07
 *
 * @copyright Copyright (c) 2025
 *
 */
#include <gtest/gtest.h>

#include <cctype>
#include <fstream>
#include <sstream>
#include <vector>

#include "orchestrator.hpp"

// Extract the function from dji_mavic_controller.cpp for testing
std::vector<RVO::Vector3> GetGoalsForLetter(char letter,
                                            const std::string &csv_path) {
  letter = std::toupper(letter);
  std::vector<RVO::Vector3> result;

  std::ifstream file(csv_path);
  if (!file.is_open()) {
    std::cerr << "Could not open CSV: " << csv_path << std::endl;
    return result;
  }

  std::string line;
  std::getline(file, line);  // skip header

  while (std::getline(file, line)) {
    std::stringstream ss(line);
    std::string token;

    char row_letter;
    float x, y, z;

    // read letter
    std::getline(ss, token, ',');
    row_letter = token[0];

    if (std::toupper(row_letter) != letter) continue;

    // read x
    std::getline(ss, token, ',');
    x = std::stof(token) - 20.0f;  // center the letter

    // read y
    std::getline(ss, token, ',');
    y = std::stof(token);

    // read z
    std::getline(ss, token, ',');
    z = std::stof(token);

    result.push_back(RVO::Vector3(x, y, z));
  }

  return result;
}

class HelperFunctionsTest : public ::testing::Test {
 protected:
  void SetUp() override {
    // Create a temporary CSV file for testing
    test_csv_path = "/tmp/test_letters.csv";
    std::ofstream file(test_csv_path);
    file << "Letter,X,Y,Z\n";
    file << "A,0.0,0.0,10.0\n";
    file << "A,1.0,0.0,10.0\n";
    file << "A,2.0,0.0,10.0\n";
    file << "B,0.0,0.0,10.0\n";
    file << "B,1.0,1.0,10.0\n";
    file << "B,2.0,1.0,10.0\n";
    file << "C,0.0,0.0,10.0\n";
    file.close();
  }

  void TearDown() override { std::remove(test_csv_path.c_str()); }

  std::string test_csv_path;
};

TEST_F(HelperFunctionsTest, GetGoalsForLetterA) {
  auto goals = GetGoalsForLetter('A', test_csv_path);
  EXPECT_EQ(goals.size(), 3);
  EXPECT_NEAR(goals[0].x(), -20.0f, 0.01f);  // X is offset by -20.0
  EXPECT_NEAR(goals[1].x(), -19.0f, 0.01f);
  EXPECT_NEAR(goals[2].x(), -18.0f, 0.01f);
  EXPECT_NEAR(goals[0].z(), 10.0f, 0.01f);
}

TEST_F(HelperFunctionsTest, GetGoalsForLetterB) {
  auto goals = GetGoalsForLetter('B', test_csv_path);
  EXPECT_EQ(goals.size(), 3);
  EXPECT_NEAR(goals[1].y(), 1.0f, 0.01f);
}

TEST_F(HelperFunctionsTest, GetGoalsForLetterCaseInsensitive) {
  auto goals_lower = GetGoalsForLetter('a', test_csv_path);
  auto goals_upper = GetGoalsForLetter('A', test_csv_path);
  EXPECT_EQ(goals_lower.size(), goals_upper.size());
  EXPECT_EQ(goals_lower.size(), 3);
}

TEST_F(HelperFunctionsTest, GetGoalsForNonExistentLetter) {
  auto goals = GetGoalsForLetter('Z', test_csv_path);
  EXPECT_EQ(goals.size(), 0);
}

TEST_F(HelperFunctionsTest, GetGoalsForInvalidFile) {
  auto goals = GetGoalsForLetter('A', "/nonexistent/file.csv");
  EXPECT_EQ(goals.size(), 0);
}

TEST_F(HelperFunctionsTest, GetGoalsForEmptyFile) {
  std::string empty_file = "/tmp/empty_test.csv";
  std::ofstream file(empty_file);
  file << "Letter,X,Y,Z\n";
  file.close();

  auto goals = GetGoalsForLetter('A', empty_file);
  EXPECT_EQ(goals.size(), 0);

  std::remove(empty_file.c_str());
}

TEST_F(HelperFunctionsTest, GetGoalsForFileWithoutHeader) {
  std::string no_header_file = "/tmp/no_header_test.csv";
  std::ofstream file(no_header_file);
  file << "A,0.0,0.0,10.0\n";
  file << "A,1.0,0.0,10.0\n";
  file.close();

  auto goals = GetGoalsForLetter('A', no_header_file);
  // Should still work, first line is treated as header and skipped
  EXPECT_EQ(goals.size(), 1);

  std::remove(no_header_file.c_str());
}

TEST_F(HelperFunctionsTest, GetGoalsForLetterC) {
  auto goals = GetGoalsForLetter('C', test_csv_path);
  EXPECT_EQ(goals.size(), 1);
  EXPECT_NEAR(goals[0].x(), -20.0f, 0.01f);
}
