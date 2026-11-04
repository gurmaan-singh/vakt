#pragma once
// Tiny argument helper shared by the command line tools. Not a general parser on purpose.

#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

namespace vakt::cli {

class Args {
 public:
  Args(int argc, char** argv, std::string usage) : usage_(std::move(usage)) {
    for (int i = 1; i < argc; ++i) a_.emplace_back(argv[i]);
    if (has("--help") || has("-h")) {
      std::cout << usage_ << "\n";
      std::exit(0);
    }
  }

  bool has(const std::string& flag) const {
    for (const auto& s : a_)
      if (s == flag) return true;
    return false;
  }

  std::string get(const std::string& flag, const std::string& fallback) const {
    for (std::size_t i = 0; i + 1 < a_.size(); ++i)
      if (a_[i] == flag) return a_[i + 1];
    return fallback;
  }

  double number(const std::string& flag, double fallback) const {
    const std::string v = get(flag, "");
    if (v.empty()) return fallback;
    try {
      return std::stod(v);
    } catch (const std::exception&) {
      fail("bad number for " + flag + ": " + v);
    }
  }

  // First argument that is neither a flag nor a flag's value.
  std::string positional(std::size_t index = 0) const {
    std::size_t seen = 0;
    for (std::size_t i = 0; i < a_.size(); ++i) {
      if (a_[i].rfind("--", 0) == 0) {
        ++i;  // skip its value (all our flags with values take exactly one)
        continue;
      }
      if (seen++ == index) return a_[i];
    }
    return {};
  }

  [[noreturn]] void fail(const std::string& msg) const {
    std::cerr << "error: " << msg << "\n\n" << usage_ << "\n";
    std::exit(2);
  }

 private:
  std::vector<std::string> a_;
  std::string usage_;
};

}  // namespace vakt::cli
