#pragma once

// CGAL 5.4's number_utils.h relies on Boost MPL's if_c arriving transitively;
// Boost 1.90 no longer supplies it through the former include chain.
#include <boost/mpl/if.hpp>

// ConicCPA2.h calls CGAL::is_zero through CGAL_NTS but omits the declaration.
// Keep Bambu's pinned source intact and make the dependencies explicit for
// modern compilers.
#include <CGAL/number_utils.h>
