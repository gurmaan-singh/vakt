#include "vakt/assignment.hpp"

#include <limits>

namespace vakt {

namespace {

// Classic O(n^2 m) Hungarian for n <= m (1-indexed internally, as in most references).
std::vector<int> hungarian_rows_le_cols(const Eigen::MatrixXd& a) {
  const int n = static_cast<int>(a.rows());
  const int m = static_cast<int>(a.cols());
  const double inf = std::numeric_limits<double>::infinity();
  std::vector<double> u(static_cast<std::size_t>(n) + 1, 0.0), v(static_cast<std::size_t>(m) + 1, 0.0);
  std::vector<int> p(static_cast<std::size_t>(m) + 1, 0), way(static_cast<std::size_t>(m) + 1, 0);
  auto at = [](auto& vec, int i) -> auto& { return vec[static_cast<std::size_t>(i)]; };

  for (int i = 1; i <= n; ++i) {
    at(p, 0) = i;
    int j0 = 0;
    std::vector<double> minv(static_cast<std::size_t>(m) + 1, inf);
    std::vector<char> used(static_cast<std::size_t>(m) + 1, 0);
    do {
      at(used, j0) = 1;
      const int i0 = at(p, j0);
      double delta = inf;
      int j1 = 0;
      for (int j = 1; j <= m; ++j) {
        if (at(used, j)) continue;
        const double cur = a(i0 - 1, j - 1) - at(u, i0) - at(v, j);
        if (cur < at(minv, j)) {
          at(minv, j) = cur;
          at(way, j) = j0;
        }
        if (at(minv, j) < delta) {
          delta = at(minv, j);
          j1 = j;
        }
      }
      for (int j = 0; j <= m; ++j) {
        if (at(used, j)) {
          at(u, at(p, j)) += delta;
          at(v, j) -= delta;
        } else {
          at(minv, j) -= delta;
        }
      }
      j0 = j1;
    } while (at(p, j0) != 0);
    do {
      const int j1 = at(way, j0);
      at(p, j0) = at(p, j1);
      j0 = j1;
    } while (j0 != 0);
  }

  std::vector<int> row_to_col(static_cast<std::size_t>(n), -1);
  for (int j = 1; j <= m; ++j)
    if (at(p, j) != 0) row_to_col[static_cast<std::size_t>(at(p, j) - 1)] = j - 1;
  return row_to_col;
}

}  // namespace

std::vector<int> solve_assignment(const Eigen::MatrixXd& cost) {
  if (cost.rows() == 0) return {};
  if (cost.cols() == 0) return std::vector<int>(static_cast<std::size_t>(cost.rows()), -1);
  if (cost.rows() <= cost.cols()) return hungarian_rows_le_cols(cost);

  const std::vector<int> col_to_row = hungarian_rows_le_cols(cost.transpose());
  std::vector<int> row_to_col(static_cast<std::size_t>(cost.rows()), -1);
  for (std::size_t c = 0; c < col_to_row.size(); ++c)
    if (col_to_row[c] >= 0) row_to_col[static_cast<std::size_t>(col_to_row[c])] = static_cast<int>(c);
  return row_to_col;
}

double assignment_cost(const Eigen::MatrixXd& cost, const std::vector<int>& row_to_col) {
  double total = 0.0;
  for (std::size_t r = 0; r < row_to_col.size(); ++r)
    if (row_to_col[r] >= 0) total += cost(static_cast<Eigen::Index>(r), row_to_col[r]);
  return total;
}

}  // namespace vakt
