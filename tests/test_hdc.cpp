#include "fuzzylib/hdc/hypervector.hpp"

#include <cmath>

#include "minitest.hpp"

using namespace fuzzylib::hdc;

TEST_CASE(random_vector_is_self_similar) {
    HDCSpace space(2000, 1);
    Hypervector v = space.random();
    CHECK_NEAR(HDCSpace::cosineSimilarity(v, v), 1.0, 1e-9);
}

TEST_CASE(two_random_vectors_are_nearly_orthogonal) {
    HDCSpace space(4000, 2);
    Hypervector a = space.random();
    Hypervector b = space.random();
    CHECK(std::fabs(HDCSpace::cosineSimilarity(a, b)) < 0.1);
}

TEST_CASE(bind_is_dissimilar_to_its_inputs) {
    HDCSpace space(4000, 3);
    Hypervector a = space.random();
    Hypervector b = space.random();
    Hypervector bound = space.bind(a, b);
    CHECK(std::fabs(HDCSpace::cosineSimilarity(bound, a)) < 0.1);
    CHECK(std::fabs(HDCSpace::cosineSimilarity(bound, b)) < 0.1);
}

TEST_CASE(bind_is_self_inverse_for_bipolar_vectors) {
    HDCSpace space(2000, 4);
    Hypervector a = space.random();
    Hypervector b = space.random();
    Hypervector bound = space.bind(a, b);
    Hypervector recovered = space.bind(bound, b);  // bind is its own inverse: (a*b)*b == a
    CHECK_NEAR(HDCSpace::cosineSimilarity(recovered, a), 1.0, 1e-9);
}

TEST_CASE(bundle_is_similar_to_all_its_inputs) {
    HDCSpace space(4000, 5);
    Hypervector a = space.random();
    Hypervector b = space.random();
    Hypervector c = space.random();
    Hypervector bundled = space.bundle({a, b, c});
    CHECK(HDCSpace::cosineSimilarity(bundled, a) > 0.3);
    CHECK(HDCSpace::cosineSimilarity(bundled, b) > 0.3);
    CHECK(HDCSpace::cosineSimilarity(bundled, c) > 0.3);
}

TEST_CASE(item_memory_finds_nearest_prototype) {
    HDCSpace space(2000, 6);
    ItemMemory memory;
    memory.add("cold", space.random());
    memory.add("hot", space.random());

    const auto [name, score] = memory.nearest(memory.get("hot"));
    CHECK(name == "hot");
    CHECK_NEAR(score, 1.0, 1e-9);
}

TEST_CASE(scalar_encoder_is_smooth_and_discriminative) {
    HDCSpace space(4000, 7);
    ScalarEncoder encoder(space, 0.0, 100.0, 20);

    const double closeSimilarity = HDCSpace::cosineSimilarity(encoder.encode(50.0), encoder.encode(52.0));
    const double farSimilarity = HDCSpace::cosineSimilarity(encoder.encode(0.0), encoder.encode(100.0));
    CHECK(closeSimilarity > farSimilarity);
    CHECK(closeSimilarity > 0.8);
    CHECK(farSimilarity < 0.2);
}

int main() { return minitest::run_all_tests(); }
