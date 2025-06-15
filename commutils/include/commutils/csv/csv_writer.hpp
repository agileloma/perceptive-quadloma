/* ----------------------------------------------------------------------------
 * Copyright (c) 2025, Harbin Institute of Technology.
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 *  @file   csv_writer.hpp
 *  @author Jun Li (junli@hit.edu.cn)
 *  @brief  CSVWriter class for writing CSV files
 *  @date   April 28, 2025
 **/

#pragma once

#include <string>
#include <vector>
#include <fstream>
#include <sstream>
#include <stdexcept>

#include <Eigen/Dense>

namespace CSV {

class CSVWriter
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    using VectorXd = Eigen::VectorXd;
    using ConstRefVectorXd = const Eigen::Ref<const VectorXd>&;

    /**
     * @brief Constructor to initialize the CSVWriter with a filename and delimiter.
     * @param filename The name of the CSV file to write to.
     * @param delimiter The delimiter to use between values (default is comma).
     */
    CSVWriter(const std::string& filename, const std::string& delimiter = ",")
        : filename_(filename), delimiter_(delimiter), lines_count_(0)
    {}

    /**
     * @brief Add a header row to the CSV file.
     * @param heading A vector of strings representing the header row.
     * @throw std::runtime_error If the file fails to open.
     */
    void addDataHeading(const std::vector<std::string>& heading)
    {
        heading_size_ = heading.size();  // Store the size for validation

        std::ofstream file;
        openFile(file);

        auto first = heading.begin();
        auto last = heading.end();
        for (; first != last; ) {
            std::string value = *first;

            // Check for special characters and escape if necessary
            if (value.find(delimiter_) != std::string::npos || 
                value.find('"') != std::string::npos) 
            {
                // Escape double quotes
                size_t pos = 0;
                while ((pos = value.find('"', pos)) != std::string::npos) {
                    value.insert(pos, 1, '"');  // Add an extra double quote
                    pos += 2;
                }
                // Enclose in double quotes
                value = "\"" + value + "\"";
            }

            file << value;
            if (++first != last)
                file << delimiter_;
        }
        file << "\n";

        lines_count_++;

        file.close();  // Close the file
    }

    /**
     * @brief Add a row of data to the CSV file.
     * @param data A vector of double values to write as a row.
     * @throw std::runtime_error If the file fails to open.
     */
    void addDataInRow(ConstRefVectorXd data)
    {
        if (heading_size_ > 0 && 
            heading_size_ != static_cast<std::size_t>(data.size())) {
            throw std::runtime_error(
                "Column count mismatch between data heading and data row");
        }

        std::ofstream file;
        openFile(file);

        std::size_t data_size = data.size();
        for (std::size_t i = 0; i < data_size; i++) {
            file << data(i);
            if (i < data_size - 1)
                file << delimiter_;
        }
        file << "\n";

        lines_count_++;

        file.close();  // Close the file
    }

private:
    std::string filename_;
    std::string delimiter_;
    int lines_count_;
    std::size_t heading_size_;  // Stores the size of the heading for validation

    /**
     * @brief Helper method to open the file in the appropriate mode.
     * @param file The file stream to open.
     * @throw std::runtime_error If the file fails to open.
     */
    void openFile(std::ofstream& file)
    {
        file.open(filename_, std::ios::out | 
                  (lines_count_ ? std::ios::app : std::ios::trunc));
        if (!file.is_open()) {
            throw std::runtime_error("Error opening file: " + filename_);
        }
    }
};

}  // namespace CSV