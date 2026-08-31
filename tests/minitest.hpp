#pragma once

// A deliberately tiny, dependency-free test harness: register test
// functions with TEST_CASE, assert with CHECK/CHECK_NEAR/CHECK_THROWS, and
// call run_all_tests() from main(). Kept in-repo so the build never needs
// network access to fetch a testing framework.

#include <cmath>
#include <cstdlib>
#include <functional>
#include <iostream>
#include <string>
#include <vector>

namespace minitest {

struct TestCase {
    std::string name;
    std::function<void()> fn;
};

inline std::vector<TestCase>& registry() {
    static std::vector<TestCase> tests;
    return tests;
}

inline int& failureCount() {
    static int count = 0;
    return count;
}

inline int& checkCount() {
    static int count = 0;
    return count;
}

struct Registrar {
    Registrar(const std::string& name, std::function<void()> fn) { registry().push_back({name, std::move(fn)}); }
};

inline void reportFailure(const std::string& file, int line, const std::string& message) {
    ++failureCount();
    std::cerr << file << ":" << line << ": FAILED: " << message << "\n";
}

inline int run_all_tests() {
    int failedTests = 0;
    for (const auto& test : registry()) {
        const int before = failureCount();
        std::cout << "[ RUN      ] " << test.name << "\n";
        try {
            test.fn();
        } catch (const std::exception& e) {
            reportFailure(__FILE__, __LINE__, "uncaught exception in '" + test.name + "': " + e.what());
        } catch (...) {
            reportFailure(__FILE__, __LINE__, "uncaught unknown exception in '" + test.name + "'");
        }
        if (failureCount() != before) {
            ++failedTests;
            std::cout << "[  FAILED  ] " << test.name << "\n";
        } else {
            std::cout << "[       OK ] " << test.name << "\n";
        }
    }
    std::cout << checkCount() << " checks run, " << failureCount() << " failed, " << failedTests
               << " test case(s) failed out of " << registry().size() << "\n";
    return failureCount() == 0 ? 0 : 1;
}

}  // namespace minitest

#define TEST_CASE(name)                                                    \
    static void name();                                                    \
    static ::minitest::Registrar registrar_##name(#name, name);            \
    static void name()

#define CHECK(cond)                                                         \
    do {                                                                    \
        ++::minitest::checkCount();                                         \
        if (!(cond)) {                                                      \
            ::minitest::reportFailure(__FILE__, __LINE__, "CHECK(" #cond ")"); \
        }                                                                    \
    } while (0)

#define CHECK_NEAR(a, b, eps)                                                                     \
    do {                                                                                          \
        ++::minitest::checkCount();                                                               \
        const double a_ = (a);                                                                    \
        const double b_ = (b);                                                                    \
        if (std::fabs(a_ - b_) > (eps)) {                                                         \
            ::minitest::reportFailure(__FILE__, __LINE__,                                         \
                                       "CHECK_NEAR(" #a ", " #b ") -> " + std::to_string(a_) +     \
                                           " vs " + std::to_string(b_));                           \
        }                                                                                          \
    } while (0)

#define CHECK_THROWS(expr)                                                                 \
    do {                                                                                   \
        ++::minitest::checkCount();                                                        \
        bool threw_ = false;                                                               \
        try {                                                                              \
            (void)(expr);                                                                  \
        } catch (...) {                                                                    \
            threw_ = true;                                                                 \
        }                                                                                  \
        if (!threw_) {                                                                     \
            ::minitest::reportFailure(__FILE__, __LINE__, "CHECK_THROWS(" #expr ")");      \
        }                                                                                   \
    } while (0)
