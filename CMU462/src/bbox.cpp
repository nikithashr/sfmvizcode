#include "bbox.h"

#include <algorithm>
#include <iostream>

namespace CMU462 {

std::ostream& operator<<(std::ostream& os, const BBox& b) {
  return os << "BBOX(" << b.min << ", " << b.max << ")";
}

} // namespace CMU462
