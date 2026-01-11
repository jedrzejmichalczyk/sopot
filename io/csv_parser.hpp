#pragma once

#include <string>
#include <vector>
#include <fstream>
#include <sstream>
#include <stdexcept>

namespace sopot::io {

/**
 * Simple CSV parser for rocket simulation data
 */
class CsvParser {
public:
    struct Table {
        std::vector<std::vector<double>> data;  // rows x cols
        size_t rows() const { return data.size(); }
        size_t cols() const { return data.empty() ? 0 : data[0].size(); }

        // Get column as vector
        std::vector<double> column(size_t col) const {
            std::vector<double> result;
            result.reserve(rows());
            for (const auto& row : data) {
                if (col < row.size()) {
                    result.push_back(row[col]);
                }
            }
            return result;
        }

        // Get row
        const std::vector<double>& row(size_t r) const {
            return data[r];
        }

        double operator()(size_t r, size_t c) const {
            return data[r][c];
        }
    };

    // Parse CSV file with given delimiter
    static Table parseFile(const std::string& filename, char delimiter = ',') {
        std::ifstream file(filename);
        if (!file.is_open()) {
            throw std::runtime_error("Cannot open file: " + filename);
        }

        Table table;
        std::string line;

        while (std::getline(file, line)) {
            // Skip empty lines
            if (line.empty() || line[0] == '#') continue;

            std::vector<double> row;
            std::stringstream ss(line);
            std::string cell;

            while (std::getline(ss, cell, delimiter)) {
                try {
                    // Trim whitespace
                    size_t start = cell.find_first_not_of(" \t\r\n");
                    size_t end = cell.find_last_not_of(" \t\r\n");
                    if (start != std::string::npos && end != std::string::npos) {
                        cell = cell.substr(start, end - start + 1);
                    }
                    if (!cell.empty()) {
                        row.push_back(std::stod(cell));
                    }
                } catch (...) {
                    // Skip non-numeric values (headers)
                }
            }

            if (!row.empty()) {
                table.data.push_back(std::move(row));
            }
        }

        return table;
    }

    // Parse 2D table (first row is column headers, first column is row headers)
    // Returns table without headers
    static Table parse2DTable(const std::string& filename, char delimiter = ',') {
        std::ifstream file(filename);
        if (!file.is_open()) {
            throw std::runtime_error("Cannot open file: " + filename);
        }

        Table table;
        std::string line;
        bool first_row = true;

        while (std::getline(file, line)) {
            if (line.empty() || line[0] == '#') continue;

            std::vector<double> row;
            std::stringstream ss(line);
            std::string cell;
            bool first_col = true;

            while (std::getline(ss, cell, delimiter)) {
                try {
                    size_t start = cell.find_first_not_of(" \t\r\n");
                    size_t end = cell.find_last_not_of(" \t\r\n");
                    if (start != std::string::npos && end != std::string::npos) {
                        cell = cell.substr(start, end - start + 1);
                    }
                    if (!cell.empty()) {
                        double value = std::stod(cell);
                        // Skip first column (row headers) except in first row
                        if (!first_col || first_row) {
                            row.push_back(value);
                        }
                    }
                } catch (...) {
                    // Skip non-numeric values
                }
                first_col = false;
            }

            if (!row.empty()) {
                table.data.push_back(std::move(row));
            }
            first_row = false;
        }

        return table;
    }
};

} // namespace sopot::io
