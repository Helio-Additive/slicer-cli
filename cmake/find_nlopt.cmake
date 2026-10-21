# NLopt for libnest2d, which links NLopt::nlopt.
#
# Ubuntu's libnlopt-dev and libnlopt-cxx-dev each install an NLoptConfig.cmake,
# in cmake/nlopt (target NLopt::nlopt) and cmake/nlopt_cxx (target
# NLopt::nlopt_cxx, library libnlopt_cxx, which the Linux package does not
# bundle). find_package matches both directories and, unless told to sort,
# takes the one the filesystem lists first. Sorted by name, cmake/nlopt comes
# first on every machine.
#
# The sort settings live in a function scope, so the caller's own settings
# for later find_package calls stay as they were. Sets NLopt_FOUND and
# NLopt_DIR. Tested by tests/test_find_nlopt.sh.
function(slicer_cli_find_nlopt)
  set(CMAKE_FIND_PACKAGE_SORT_ORDER NAME)
  set(CMAKE_FIND_PACKAGE_SORT_DIRECTION ASC)
  find_package(NLopt QUIET)
  set(NLopt_FOUND "${NLopt_FOUND}" PARENT_SCOPE)
endfunction()
slicer_cli_find_nlopt()
if(NLopt_FOUND AND NOT TARGET NLopt::nlopt)
  message(FATAL_ERROR
    "NLopt was found at ${NLopt_DIR}, but that package has no NLopt::nlopt target, "
    "which libnest2d links. Set NLopt_DIR to the C package's config directory "
    "(cmake/nlopt, from libnlopt-dev), or clear a cached NLopt_DIR.")
endif()
