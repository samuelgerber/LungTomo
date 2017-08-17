cmake_minimum_required (VERSION 2.8)

# This project is designed to be built outside the RTK source tree.
project( LungTomo )

# Find the RTK libraries and includes
find_package(RTK REQUIRED)
include(${RTK_USE_FILE})

# Executable
add_executable(ForwardProject ForwardProject.cxx )
target_link_libraries(ForwardProject ${RTK_LIBRARIES})
target_link_libraries(ForwardProject ${ITK_LIBRARIES})


add_executable(HighResCT HighResCT.cxx )
target_link_libraries(HighResCT ${RTK_LIBRARIES})
target_link_libraries(HighResCT ${ITK_LIBRARIES})
