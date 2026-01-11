#pragma once

#include "../core/scalar.hpp"
#include <vector>
#include <algorithm>
#include <stdexcept>
#include <cmath>

namespace sopot::io {

/**
 * 1D Linear Interpolation
 */
template<Scalar T = double>
class LinearInterpolator {
private:
    std::vector<double> m_x;  // x values (always double for indexing)
    std::vector<double> m_y;  // y values (always double for storage)
    std::string m_name;

public:
    LinearInterpolator() = default;

    LinearInterpolator(
        const std::vector<double>& x,
        const std::vector<double>& y,
        std::string name = ""
    ) : m_x(x), m_y(y), m_name(std::move(name)) {
        if (m_x.size() != m_y.size()) {
            throw std::runtime_error("Interpolator: x and y must have same size");
        }
        if (m_x.size() < 2) {
            throw std::runtime_error("Interpolator: need at least 2 points");
        }
    }

    void setup(const std::vector<double>& x, const std::vector<double>& y, std::string name = "") {
        m_x = x;
        m_y = y;
        m_name = std::move(name);
    }

    bool empty() const { return m_x.empty(); }
    size_t size() const { return m_x.size(); }
    double xMin() const { return m_x.front(); }
    double xMax() const { return m_x.back(); }

    // Interpolate at point
    T interpolate(T x_val) const {
        if (m_x.empty()) return T(0);

        double x = value_of(x_val);

        // Clamp to range
        if (x <= m_x.front()) return T(m_y.front());
        if (x >= m_x.back()) return T(m_y.back());

        // Binary search for interval
        auto it = std::lower_bound(m_x.begin(), m_x.end(), x);
        size_t i = std::distance(m_x.begin(), it);
        if (i > 0) --i;

        // Linear interpolation
        double x0 = m_x[i];
        double x1 = m_x[i + 1];
        double y0 = m_y[i];
        double y1 = m_y[i + 1];

        double t = (x - x0) / (x1 - x0);
        return T(y0 + t * (y1 - y0));
    }

    T operator()(T x) const { return interpolate(x); }
};


/**
 * 2D Bilinear Interpolation (for aero coefficients)
 * Interpolates on a (row, col) grid, e.g., (AoA, Mach)
 */
template<Scalar T = double>
class BilinearInterpolator {
private:
    std::vector<double> m_rows;   // Row values (e.g., AoA)
    std::vector<double> m_cols;   // Column values (e.g., Mach)
    std::vector<std::vector<double>> m_data;  // data[row][col]
    std::string m_name;

public:
    BilinearInterpolator() = default;

    // Setup from 2D table where first row is column headers (Mach),
    // first column is row headers (AoA), rest is data
    void setupFromTable(const std::vector<std::vector<double>>& table, std::string name = "") {
        m_name = std::move(name);

        if (table.empty() || table[0].empty()) {
            throw std::runtime_error("BilinearInterpolator: empty table");
        }

        // First row contains column headers (skip first cell)
        m_cols.clear();
        for (size_t j = 1; j < table[0].size(); ++j) {
            m_cols.push_back(table[0][j]);
        }

        // Rest of rows: first cell is row header, rest is data
        m_rows.clear();
        m_data.clear();
        for (size_t i = 1; i < table.size(); ++i) {
            if (table[i].empty()) continue;
            m_rows.push_back(table[i][0]);
            std::vector<double> row_data;
            for (size_t j = 1; j < table[i].size(); ++j) {
                row_data.push_back(table[i][j]);
            }
            m_data.push_back(std::move(row_data));
        }
    }

    // Setup directly
    void setup(
        const std::vector<double>& rows,
        const std::vector<double>& cols,
        const std::vector<std::vector<double>>& data,
        std::string name = ""
    ) {
        m_rows = rows;
        m_cols = cols;
        m_data = data;
        m_name = std::move(name);
    }

    bool empty() const { return m_data.empty(); }

    // Bilinear interpolation at (row_val, col_val)
    T interpolate(T row_val, T col_val) const {
        if (m_data.empty()) return T(0);

        double row = value_of(row_val);
        double col = value_of(col_val);

        // Clamp row
        if (row <= m_rows.front()) row = m_rows.front();
        else if (row >= m_rows.back()) row = m_rows.back();

        // Clamp col
        if (col <= m_cols.front()) col = m_cols.front();
        else if (col >= m_cols.back()) col = m_cols.back();

        // Find row interval
        auto row_it = std::lower_bound(m_rows.begin(), m_rows.end(), row);
        size_t ri = std::distance(m_rows.begin(), row_it);
        if (ri > 0) --ri;
        if (ri >= m_rows.size() - 1) ri = m_rows.size() - 2;

        // Find col interval
        auto col_it = std::lower_bound(m_cols.begin(), m_cols.end(), col);
        size_t ci = std::distance(m_cols.begin(), col_it);
        if (ci > 0) --ci;
        if (ci >= m_cols.size() - 1) ci = m_cols.size() - 2;

        // Bilinear interpolation
        double r0 = m_rows[ri];
        double r1 = m_rows[ri + 1];
        double c0 = m_cols[ci];
        double c1 = m_cols[ci + 1];

        double tr = (row - r0) / (r1 - r0);
        double tc = (col - c0) / (c1 - c0);

        double v00 = m_data[ri][ci];
        double v01 = m_data[ri][ci + 1];
        double v10 = m_data[ri + 1][ci];
        double v11 = m_data[ri + 1][ci + 1];

        double v0 = v00 + tc * (v01 - v00);
        double v1 = v10 + tc * (v11 - v10);

        return T(v0 + tr * (v1 - v0));
    }

    T operator()(T row, T col) const { return interpolate(row, col); }
};

} // namespace sopot::io
