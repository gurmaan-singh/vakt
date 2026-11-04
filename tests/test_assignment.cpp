#include "vakt/assignment.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <numeric>
#include <random>

using namespace vakt;

namespace {

// Exhaustive search over permutations; fine for <= 7 columns.
double brute_force(const Eigen::MatrixXd& c) {
  const auto rows = static_cast<int>(c.rows());
  const auto cols = static_cast<int>(c.cols());
  std::vector<int> perm(static_cast<std::size_t>(std::max(rows, cols)));
  std::iota(perm.begin(), perm.end(), 0);
  double best = std::numeric_limits<double>::infinity();
  do {
    double s = 0.0;
    int used = 0;
    for (int r = 0; r < rows; ++r) {
      const int col = perm[static_cast<std::size_t>(r)];
      if (col < cols) {
        s += c(r, col);
        ++used;
      }
    }
    if (used == std::min(rows, cols)) best = std::min(best, s);
  } while (std::next_permutation(perm.begin(), perm.end()));
  return best;
}

}  // namespace

TEST(Assignment, TextbookExample) {
  Eigen::MatrixXd c(3, 3);
  c << 4, 1, 3,
       2, 0, 5,
       3, 2, 2;
  const auto a = solve_assignment(c);
  EXPECT_EQ(a, (std::vector<int>{1, 0, 2}));
  EXPECT_DOUBLE_EQ(assignment_cost(c, a), 5.0);
}

TEST(Assignment, EmptyAndDegenerate) {
  EXPECT_TRUE(solve_assignment(Eigen::MatrixXd(0, 4)).empty());
  EXPECT_EQ(solve_assignment(Eigen::MatrixXd(2, 0)), (std::vector<int>{-1, -1}));
}

TEST(Assignment, MoreRowsThanColumnsLeavesRowsOut) {
  Eigen::MatrixXd c(3, 1);
  c << 5, 1, 3;
  EXPECT_EQ(solve_assignment(c), (std::vector<int>{-1, 0, -1}));
}

TEST(Assignment, MatchesBruteForceOnRandomMatrices) {
  std::mt19937 rng(42);
  std::uniform_real_distribution<double> u(0.0, 100.0);
  std::uniform_int_distribution<int> dim(1, 6);
  for (int trial = 0; trial < 500; ++trial) {
    Eigen::MatrixXd c(dim(rng), dim(rng));
    for (Eigen::Index i = 0; i < c.size(); ++i) c.data()[i] = u(rng);
    const auto a = solve_assignment(c);
    // every column used at most once
    std::vector<int> seen;
    for (int col : a)
      if (col >= 0) seen.push_back(col);
    std::sort(seen.begin(), seen.end());
    ASSERT_TRUE(std::adjacent_find(seen.begin(), seen.end()) == seen.end());
    ASSERT_EQ(seen.size(), static_cast<std::size_t>(std::min(c.rows(), c.cols())));
    ASSERT_NEAR(assignment_cost(c, a), brute_force(c), 1e-9) << "trial " << trial << "\n" << c;
  }
}

TEST(Assignment, LargeForbiddenCostsDoNotOverflow) {
  // The tracker uses 1e12 for "outside the gate"; make sure it still picks the real pair.
  Eigen::MatrixXd c = Eigen::MatrixXd::Constant(3, 5, 1e12);
  c(0, 3) = 2.0;
  c(1, 0) = 1.0;
  c(2, 4) = 7.0;
  EXPECT_EQ(solve_assignment(c), (std::vector<int>{3, 0, 4}));
}
