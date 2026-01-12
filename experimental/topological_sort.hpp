#pragma once

#include <array>
#include <cstddef>

namespace sopot::experimental {

// ============================================================================
// TOPOLOGICAL SORT - Kahn's algorithm (compile-time)
// ============================================================================

template<size_t N>
struct TopologicalSortResult {
    std::array<size_t, N> order;
    bool hasCycle;
    size_t count;  // Number of nodes processed
};

template<size_t N>
constexpr TopologicalSortResult<N> topologicalSort(
    const std::array<std::array<bool, N>, N>& adj
) {
    TopologicalSortResult<N> result{};
    result.hasCycle = false;
    result.count = 0;

    std::array<bool, N> visited{};
    std::array<int, N> inDegree{};

    // Compute in-degrees
    for (size_t j = 0; j < N; ++j) {
        for (size_t i = 0; i < N; ++i) {
            if (adj[i][j]) {
                inDegree[i]++;
            }
        }
    }

    // Process nodes with in-degree 0
    for (size_t iter = 0; iter < N; ++iter) {
        // Find a node with in-degree 0
        bool found = false;
        for (size_t i = 0; i < N; ++i) {
            if (!visited[i] && inDegree[i] == 0) {
                // Add to result
                result.order[result.count++] = i;
                visited[i] = true;
                found = true;

                // Decrease in-degree of dependent nodes
                for (size_t j = 0; j < N; ++j) {
                    if (adj[j][i]) {
                        inDegree[j]--;
                    }
                }
                break;
            }
        }

        if (!found) {
            // No node with in-degree 0 found
            // Either done or cycle detected
            if (result.count < N) {
                result.hasCycle = true;
            }
            break;
        }
    }

    return result;
}

// ============================================================================
// CYCLE DETECTION - DFS-based
// ============================================================================

template<size_t N>
constexpr bool hasCycle(const std::array<std::array<bool, N>, N>& adj) {
    auto result = topologicalSort<N>(adj);
    return result.hasCycle;
}

// ============================================================================
// EXECUTION ORDER - Get topologically sorted order
// ============================================================================

template<size_t N>
constexpr std::array<size_t, N> getExecutionOrder(
    const std::array<std::array<bool, N>, N>& adj
) {
    auto result = topologicalSort<N>(adj);

    // If cycle detected, return empty (all zeros)
    if (result.hasCycle) {
        return {};
    }

    return result.order;
}

}  // namespace sopot::experimental
