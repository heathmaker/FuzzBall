#include "fuzzylib/core/membership.hpp"

#include "minitest.hpp"

using namespace fuzzylib;

TEST_CASE(triangular_basic_shape) {
    mf::Triangular tri(0.0, 5.0, 10.0);
    CHECK_NEAR(tri(0.0), 0.0, 1e-9);
    CHECK_NEAR(tri(5.0), 1.0, 1e-9);
    CHECK_NEAR(tri(10.0), 0.0, 1e-9);
    CHECK_NEAR(tri(2.5), 0.5, 1e-9);
    CHECK_NEAR(tri(7.5), 0.5, 1e-9);
    CHECK_NEAR(tri(-1.0), 0.0, 1e-9);
    CHECK_NEAR(tri(11.0), 0.0, 1e-9);
}

TEST_CASE(triangular_rejects_invalid_order) { CHECK_THROWS(mf::Triangular(5.0, 0.0, 10.0)); }

TEST_CASE(trapezoidal_basic_shape) {
    mf::Trapezoidal trap(0.0, 2.0, 8.0, 10.0);
    CHECK_NEAR(trap(0.0), 0.0, 1e-9);
    CHECK_NEAR(trap(1.0), 0.5, 1e-9);
    CHECK_NEAR(trap(5.0), 1.0, 1e-9);
    CHECK_NEAR(trap(9.0), 0.5, 1e-9);
    CHECK_NEAR(trap(10.0), 0.0, 1e-9);
}

TEST_CASE(gaussian_peaks_at_mean) {
    mf::Gaussian g(3.0, 1.0);
    CHECK_NEAR(g(3.0), 1.0, 1e-9);
    CHECK(g(3.0) > g(4.0));
    CHECK(g(4.0) > g(6.0));
}

TEST_CASE(sshape_and_zshape_are_complementary_at_endpoints) {
    mf::SShape s(0.0, 10.0);
    mf::ZShape z(0.0, 10.0);
    CHECK_NEAR(s(0.0), 0.0, 1e-9);
    CHECK_NEAR(s(10.0), 1.0, 1e-9);
    CHECK_NEAR(z(0.0), 1.0, 1e-9);
    CHECK_NEAR(z(10.0), 0.0, 1e-9);
    for (double x = 0.0; x <= 10.0; x += 1.0) {
        CHECK_NEAR(s(x) + z(x), 1.0, 1e-9);
    }
}

TEST_CASE(trapezoidal_degenerate_shoulder_holds_at_universe_boundary) {
    // A right shoulder pinned to the top of the universe (c == d) must
    // stay fully membered at x == d, not drop to 0: this is how an
    // open-ended "high"/"good" term is conventionally modeled.
    mf::Trapezoidal highShoulder(60.0, 80.0, 100.0, 100.0);
    CHECK_NEAR(highShoulder(100.0), 1.0, 1e-9);
    CHECK_NEAR(highShoulder(80.0), 1.0, 1e-9);

    // Symmetric case on the left: a == b pinned to the bottom.
    mf::Trapezoidal lowShoulder(0.0, 0.0, 20.0, 45.0);
    CHECK_NEAR(lowShoulder(0.0), 1.0, 1e-9);
}

TEST_CASE(singleton_only_matches_value) {
    mf::Singleton s(4.2);
    CHECK_NEAR(s(4.2), 1.0, 1e-9);
    CHECK_NEAR(s(4.3), 0.0, 1e-9);
}

int main() { return minitest::run_all_tests(); }
