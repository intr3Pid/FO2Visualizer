#include "sphere_info.h"

using namespace std;

namespace CMU462 { namespace Collada {

ostream& operator<<( ostream& os, const SphereInfo& sphere ) {
  return os << "Sphere: " << sphere.name << " (id:" << sphere.id << ") ["
            << " radius=" << sphere.radius << " ]";
}

} // namespace Collada
} // namespace CMU462
