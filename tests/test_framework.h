#ifndef TEST_FRAMEWORK_H
#define TEST_FRAMEWORK_H

#include <iostream>
#include <string>
#include <exception>

namespace test {

inline int passed = 0;
inline int failed = 0;
inline std::string current_suite;

inline void suite(const std::string& name) {
    current_suite = name;
    std::cout << "\n=== " << name << " ===\n";
}

inline void check(bool condition, const std::string& label) {
    if (condition) {
        std::cout << "  [PASS] " << label << "\n";
        ++passed;
    } else {
        std::cout << "  [FAIL] " << label << "  <-- FAILED\n";
        ++failed;
    }
}

template<typename Fn>
inline void throws(Fn&& fn, const std::string& label) {
    bool caught = false;
    try { fn(); } catch (...) { caught = true; }
    check(caught, label + " (expected exception)");
}

template<typename Fn>
inline void no_throw(Fn&& fn, const std::string& label) {
    bool ok = true;
    try { fn(); } catch (std::exception& e) {
        ok = false;
        std::cout << "  [FAIL] " << label << "  -- exception: " << e.what() << "\n";
        ++failed;
        return;
    }
    check(ok, label + " (no exception)");
}

inline void report() {
    std::cout << "\n--------------------------------------------\n";
    std::cout << "  Results: " << passed << " passed, " << failed << " failed\n";
    std::cout << "--------------------------------------------\n";
}

} // namespace test

#endif // TEST_FRAMEWORK_H
