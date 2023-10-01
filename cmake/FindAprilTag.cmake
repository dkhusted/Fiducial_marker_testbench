##################################################################################################
# 
# CMake script for finding AprilTag.
# For information see FindTinyXML.cmake, as this is a copy of that cmake but made to find AprilTag
# 
##################################################################################################

# Get package location hint from environment variable (if any)
if(NOT AprilTag_ROOT_DIR AND DEFINED ENV{AprilTag_ROOT_DIR})
  set(AprilTag_ROOT_DIR "$ENV{AprilTag_ROOT_DIR}" CACHE PATH
      "AprilTag base directory location (optional, used for nonstandard installation paths)")
endif()

# Search path for nonstandard package locations
if(AprilTag_ROOT_DIR)
  set(AprilTag_INCLUDE_PATH PATHS "${AprilTag_ROOT_DIR}/include" NO_DEFAULT_PATH)
  set(AprilTag_LIBRARY_PATH PATHS "${AprilTag_ROOT_DIR}/lib"     NO_DEFAULT_PATH)
endif()

# Find headers and libraries
find_path(AprilTag_INCLUDE_DIR NAMES apriltag.h PATH_SUFFIXES "apriltag" ${AprilTag_INCLUDE_PATH})
find_library(AprilTag_LIBRARY  NAMES apriltag   PATH_SUFFIXES "apriltag" ${AprilTag_LIBRARY_PATH})

mark_as_advanced(AprilTag_INCLUDE_DIR
                 AprilTag_LIBRARY)

# Output variables generation
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(AprilTag DEFAULT_MSG AprilTag_LIBRARY
                                                      AprilTag_INCLUDE_DIR)

set(AprilTag_FOUND ${APRILTAG_FOUND}) # Enforce case-correctness: Set appropriately cased variable...
unset(APRILTAG_FOUND) # ...and unset uppercase variable generated by find_package_handle_standard_args

if(AprilTag_FOUND)
  set(AprilTag_INCLUDE_DIRS ${AprilTag_INCLUDE_DIR})
  set(AprilTag_LIBRARIES ${AprilTag_LIBRARY})
endif()
