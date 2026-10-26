#pragma once

#include <cstddef>
#include <vector>

namespace iso_detail {

// Rectangular minimum-cost assignment; costs must be nonempty with rows <= columns.
std::vector<std::size_t> minimum_assignment(const std::vector<std::vector<double>>& costs);

} // namespace iso_detail
