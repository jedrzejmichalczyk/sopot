#include "physics/connected_masses/connectivity_matrix.hpp"
#include <iostream>
#include <cassert>
#include <stdexcept>

using namespace sopot;
using namespace sopot::connected_masses;

/**
 * @brief Test helper to verify exception is thrown
 */
template<typename Func>
bool expectsException(Func&& func, const std::string& expected_msg_substring = "") {
    try {
        func();
        return false;  // No exception thrown
    } catch (const std::exception& e) {
        if (!expected_msg_substring.empty()) {
            std::string msg = e.what();
            if (msg.find(expected_msg_substring) == std::string::npos) {
                std::cerr << "Exception message doesn't contain expected substring.\n";
                std::cerr << "  Expected substring: " << expected_msg_substring << "\n";
                std::cerr << "  Actual message: " << msg << "\n";
                return false;
            }
        }
        return true;  // Expected exception thrown
    }
}

/**
 * @brief Test 1: Validate mass parameter validation
 */
void test_mass_parameter_validation() {
    std::cout << "\n=== Test 1: Mass Parameter Validation ===\n";

    // Test zero mass (should throw)
    std::cout << "Testing zero mass...";
    assert(expectsException([]() {
        IndexedPointMass<0, double>(0.0);
    }, "Mass must be positive"));
    std::cout << " ✓\n";

    // Test negative mass (should throw)
    std::cout << "Testing negative mass...";
    assert(expectsException([]() {
        IndexedPointMass<0, double>(-1.0);
    }, "Mass must be positive"));
    std::cout << " ✓\n";

    // Test positive mass (should succeed)
    std::cout << "Testing positive mass...";
    try {
        IndexedPointMass<0, double>(1.0);
        std::cout << " ✓\n";
    } catch (...) {
        std::cerr << " ✗ Unexpected exception\n";
        throw;
    }

    std::cout << "✓ All mass parameter validation tests passed\n";
}

/**
 * @brief Test 2: Validate spring parameter validation
 */
void test_spring_parameter_validation() {
    std::cout << "\n=== Test 2: Spring Parameter Validation ===\n";

    // Test zero stiffness (should throw)
    std::cout << "Testing zero stiffness...";
    assert(expectsException([]() {
        IndexedSpring<0, 1, double>(0.0, 1.0);
    }, "stiffness must be positive"));
    std::cout << " ✓\n";

    // Test negative stiffness (should throw)
    std::cout << "Testing negative stiffness...";
    assert(expectsException([]() {
        IndexedSpring<0, 1, double>(-10.0, 1.0);
    }, "stiffness must be positive"));
    std::cout << " ✓\n";

    // Test negative rest length (should throw)
    std::cout << "Testing negative rest length...";
    assert(expectsException([]() {
        IndexedSpring<0, 1, double>(10.0, -1.0);
    }, "rest length must be non-negative"));
    std::cout << " ✓\n";

    // Test negative damping (should throw)
    std::cout << "Testing negative damping...";
    assert(expectsException([]() {
        IndexedSpring<0, 1, double>(10.0, 1.0, -0.5);
    }, "damping must be non-negative"));
    std::cout << " ✓\n";

    // Test valid parameters (should succeed)
    std::cout << "Testing valid parameters...";
    try {
        IndexedSpring<0, 1, double>(10.0, 1.0, 0.5);
        std::cout << " ✓\n";
    } catch (...) {
        std::cerr << " ✗ Unexpected exception\n";
        throw;
    }

    std::cout << "✓ All spring parameter validation tests passed\n";
}

/**
 * @brief Test 3: Validate edge list validation (self-loops)
 */
void test_self_loop_detection() {
    std::cout << "\n=== Test 3: Self-Loop Detection ===\n";

    // Self-loop (mass 0 to mass 0)
    std::cout << "Testing self-loop detection...";
    constexpr auto edges_selfloop = std::array{
        std::pair{size_t(0), size_t(0)}
    };

    assert(expectsException([&]() {
        makeConnectedMassSystem<double, 2, edges_selfloop>(
            {{{1.0, 0.0, 0.0}, {1.0, 1.0, 0.0}}},
            {{{10.0, 1.0, 0.5}}}
        );
    }, "Self-loop detected"));
    std::cout << " ✓\n";

    std::cout << "✓ Self-loop detection test passed\n";
}

/**
 * @brief Test 4: Validate edge list validation (out-of-range indices)
 */
void test_out_of_range_indices() {
    std::cout << "\n=== Test 4: Out-of-Range Index Detection ===\n";

    // First index out of range
    std::cout << "Testing first index out of range...";
    constexpr auto edges_oob1 = std::array{
        std::pair{size_t(3), size_t(1)}  // Only 2 masses, so index 3 is invalid
    };

    assert(expectsException([&]() {
        makeConnectedMassSystem<double, 2, edges_oob1>(
            {{{1.0, 0.0, 0.0}, {1.0, 1.0, 0.0}}},
            {{{10.0, 1.0, 0.5}}}
        );
    }, "out-of-range"));
    std::cout << " ✓\n";

    // Second index out of range
    std::cout << "Testing second index out of range...";
    constexpr auto edges_oob2 = std::array{
        std::pair{size_t(0), size_t(5)}  // Only 2 masses, so index 5 is invalid
    };

    assert(expectsException([&]() {
        makeConnectedMassSystem<double, 2, edges_oob2>(
            {{{1.0, 0.0, 0.0}, {1.0, 1.0, 0.0}}},
            {{{10.0, 1.0, 0.5}}}
        );
    }, "out-of-range"));
    std::cout << " ✓\n";

    std::cout << "✓ Out-of-range index detection tests passed\n";
}

/**
 * @brief Test 5: Validate duplicate edge detection
 */
void test_duplicate_edge_detection() {
    std::cout << "\n=== Test 5: Duplicate Edge Detection ===\n";

    // Duplicate edges (same pair twice)
    std::cout << "Testing duplicate edges (0,1) and (0,1)...";
    constexpr auto edges_dup1 = std::array{
        std::pair{size_t(0), size_t(1)},
        std::pair{size_t(0), size_t(1)}
    };

    assert(expectsException([&]() {
        makeConnectedMassSystem<double, 2, edges_dup1>(
            {{{1.0, 0.0, 0.0}, {1.0, 1.0, 0.0}}},
            {{{10.0, 1.0, 0.5}, {10.0, 1.0, 0.5}}}
        );
    }, "Duplicate edge"));
    std::cout << " ✓\n";

    // Duplicate edges (reversed pairs)
    std::cout << "Testing duplicate edges (0,1) and (1,0)...";
    constexpr auto edges_dup2 = std::array{
        std::pair{size_t(0), size_t(1)},
        std::pair{size_t(1), size_t(0)}
    };

    assert(expectsException([&]() {
        makeConnectedMassSystem<double, 2, edges_dup2>(
            {{{1.0, 0.0, 0.0}, {1.0, 1.0, 0.0}}},
            {{{10.0, 1.0, 0.5}, {10.0, 1.0, 0.5}}}
        );
    }, "Duplicate edge"));
    std::cout << " ✓\n";

    std::cout << "✓ Duplicate edge detection tests passed\n";
}

/**
 * @brief Test 6: Matrix symmetry validation
 */
void test_matrix_symmetry_validation() {
    std::cout << "\n=== Test 6: Matrix Symmetry Validation ===\n";

    // Asymmetric matrix
    std::cout << "Testing asymmetric matrix...";
    bool asymmetric_matrix[3][3] = {
        {false, true,  false},
        {false, false, true},   // [1][0] != [0][1]
        {false, true,  false}
    };

    assert(expectsException([&]() {
        matrixToEdges<3>(asymmetric_matrix);
    }, "not symmetric"));
    std::cout << " ✓\n";

    // Matrix with diagonal (self-loop)
    std::cout << "Testing matrix with diagonal element...";
    bool diagonal_matrix[3][3] = {
        {true,  true,  false},  // Diagonal element [0][0] is true
        {true,  false, true},
        {false, true,  false}
    };

    assert(expectsException([&]() {
        matrixToEdges<3>(diagonal_matrix);
    }, "Self-loop detected"));
    std::cout << " ✓\n";

    // Valid symmetric matrix
    std::cout << "Testing valid symmetric matrix...";
    bool valid_matrix[3][3] = {
        {false, true,  false},
        {true,  false, true},
        {false, true,  false}
    };

    try {
        auto edges = matrixToEdges<3>(valid_matrix);
        assert(edges.size() == 2);  // Should have 2 edges: (0,1) and (1,2)
        std::cout << " ✓\n";
    } catch (...) {
        std::cerr << " ✗ Unexpected exception\n";
        throw;
    }

    std::cout << "✓ Matrix symmetry validation tests passed\n";
}

/**
 * @brief Test 7: Valid system creation still works
 */
void test_valid_system_creation() {
    std::cout << "\n=== Test 7: Valid System Creation ===\n";

    constexpr auto edges = std::array{
        std::pair{size_t(0), size_t(1)},
        std::pair{size_t(1), size_t(2)}
    };

    std::cout << "Creating valid 3-mass chain system...";
    try {
        auto system = makeConnectedMassSystem<double, 3, edges>(
            {{{1.0, 0.0, 0.0}, {1.5, 1.0, 0.0}, {2.0, 2.0, 0.0}}},
            {{{10.0, 1.0, 0.1}, {15.0, 1.0, 0.2}}}
        );

        // Verify system works
        auto state = system.getInitialState();
        auto derivs = system.computeDerivatives(0.0, state);
        std::cout << " ✓\n";
    } catch (...) {
        std::cerr << " ✗ Unexpected exception\n";
        throw;
    }

    std::cout << "✓ Valid system creation test passed\n";
}

int main() {
    std::cout << "Connected Masses Validation Test Suite\n";
    std::cout << "=======================================\n";
    std::cout << "\nTesting parameter validation and error handling\n";

    try {
        test_mass_parameter_validation();
        test_spring_parameter_validation();
        test_self_loop_detection();
        test_out_of_range_indices();
        test_duplicate_edge_detection();
        test_matrix_symmetry_validation();
        test_valid_system_creation();

        std::cout << "\n==========================================\n";
        std::cout << "All validation tests passed successfully! ✓\n";
        std::cout << "==========================================\n";

        return 0;
    }
    catch (const std::exception& e) {
        std::cerr << "\n✗ Test failed with exception: " << e.what() << "\n";
        return 1;
    }
}
