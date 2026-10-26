#include "assignment.h"

#include <limits>

namespace iso_detail {

// Rectangular Hungarian assignment (rows <= columns). Dummy columns allow
// missing base parts; each real column can be used at most once. Unlike greedy
// pairs, this minimizes the cost of the complete assignment.
std::vector<std::size_t> minimum_assignment(const std::vector<std::vector<double>>& costs) {
    const std::size_t rows = costs.size(), columns = costs.front().size();
    std::vector<double> u(rows + 1), v(columns + 1);
    std::vector<std::size_t> owner(columns + 1), previous(columns + 1);
    for (std::size_t row = 1; row <= rows; ++row) {
        owner[0] = row;
        std::size_t column = 0;
        std::vector<double> minimum(columns + 1, std::numeric_limits<double>::infinity());
        std::vector<bool> used(columns + 1, false);
        do {
            used[column] = true;
            const std::size_t active_row = owner[column];
            double delta = std::numeric_limits<double>::infinity();
            std::size_t next = 0;
            for (std::size_t j = 1; j <= columns; ++j) {
                if (used[j]) continue;
                const double reduced = costs[active_row - 1][j - 1] - u[active_row] - v[j];
                if (reduced < minimum[j]) {
                    minimum[j] = reduced;
                    previous[j] = column;
                }
                if (minimum[j] < delta) {
                    delta = minimum[j];
                    next = j;
                }
            }
            for (std::size_t j = 0; j <= columns; ++j) {
                if (used[j]) {
                    u[owner[j]] += delta;
                    v[j] -= delta;
                } else minimum[j] -= delta;
            }
            column = next;
        } while (owner[column] != 0);
        do {
            const std::size_t next = previous[column];
            owner[column] = owner[next];
            column = next;
        } while (column != 0);
    }
    std::vector<std::size_t> assignment(rows);
    for (std::size_t j = 1; j <= columns; ++j) {
        if (owner[j] != 0) assignment[owner[j] - 1] = j - 1;
    }
    return assignment;
}

} // namespace iso_detail
