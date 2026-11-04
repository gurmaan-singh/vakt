#pragma once

#include <Eigen/Dense>

#include <vector>

namespace vakt {

// Minimum-cost assignment of rows to columns (Hungarian / Kuhn-Munkres, O(n^2 m) with
// row potentials and shortest augmenting paths). Works for any rows x cols shape; if
// rows > cols some rows stay unassigned.
//
// Returns, for each row, the assigned column or -1.
std::vector<int> solve_assignment(const Eigen::MatrixXd& cost);

// Total cost of an assignment, for tests and diagnostics.
double assignment_cost(const Eigen::MatrixXd& cost, const std::vector<int>& row_to_col);

}  // namespace vakt
