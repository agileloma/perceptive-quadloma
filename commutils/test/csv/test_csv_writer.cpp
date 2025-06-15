/* ----------------------------------------------------------------------------
 * Copyright (c) 2025, Harbin Institute of Technology.
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   test_csv_writer.cpp
 * @author Jun Li (junli@hit.edu.cn)
 * @brief  Test file for CSVWriter class
 * @date   April 28, 2025
 **/

#include "commutils/csv/csv_writer.hpp"

#include <gtest/gtest.h>
#include <fstream> // For validating file contents

class CSVWriterTest : public ::testing::Test {
protected:
    void SetUp() override {}
    void TearDown() override {
        // Cleanup temporary files created during tests
        std::remove("test_data.csv");
        std::remove("existing_file.csv");
    }
};

// Test: Validate adding data headings and a row
TEST_F(CSVWriterTest, AddingDataHeadingAndRow) {
    using namespace CSV;

    CSVWriter writer("test_data.csv");

    // Define and add data headings
    std::vector<std::string> data_headings = {
        "time", "data"
    };
    writer.addDataHeading(data_headings);

    // Define and add a row of data
    Eigen::VectorXd data(data_headings.size());
    data << 0.0, 1.0;
    writer.addDataInRow(data);

    // Validate file content
    std::ifstream infile("test_data.csv");
    ASSERT_TRUE(infile.is_open());

    std::string line;
    std::getline(infile, line);
    EXPECT_EQ(line, "time,data");

    std::getline(infile, line);
    EXPECT_EQ(line, "0,1");

    infile.close();
}

// Test: Handle writing to an existing file
TEST_F(CSVWriterTest, WritingToExistingFile) {
    using namespace CSV;

    // Create an existing file
    std::ofstream outfile("existing_file.csv");
    outfile << "existing,data\n";
    outfile.close();

    CSVWriter writer("existing_file.csv");

    // Define and add data headings
    std::vector<std::string> data_headings = {"time", "data"};
    writer.addDataHeading(data_headings);

    // Define and add a row of data
    Eigen::VectorXd data(data_headings.size());
    data << 2.0, 3.0;
    writer.addDataInRow(data);

    // Validate file content
    std::ifstream infile("existing_file.csv");
    ASSERT_TRUE(infile.is_open());

    std::string line;
    std::getline(infile, line);
    EXPECT_EQ(line, "time,data");

    std::getline(infile, line);
    EXPECT_EQ(line, "2,3");

    infile.close();
}

// Test: Handle mismatched column count in data row
TEST_F(CSVWriterTest, MismatchedColumnCount) {
    using namespace CSV;

    CSVWriter writer("test_data.csv");

    // Define and add data headings
    std::vector<std::string> data_headings = {"time", "data"};
    writer.addDataHeading(data_headings);

    // Define mismatched data row
    Eigen::VectorXd data(data_headings.size() + 1);
    data << 0.0, 1.0, 2.0;

    // Expect an exception or error
    EXPECT_ANY_THROW(writer.addDataInRow(data));
}

// Test: Validate handling of special characters in headings
TEST_F(CSVWriterTest, SpecialCharactersInHeadings) {
    using namespace CSV;

    CSVWriter writer("test_data.csv");

    // Define and add data headings
    std::vector<std::string> data_headings = {"time", "data!with,commas"};
    writer.addDataHeading(data_headings);

    // Validate file content
    std::ifstream infile("test_data.csv");
    ASSERT_TRUE(infile.is_open());

    std::string line;
    std::getline(infile, line);
    EXPECT_EQ(line, "time,\"data!with,commas\"");

    infile.close();
}