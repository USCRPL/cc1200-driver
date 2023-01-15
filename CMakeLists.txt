set(CC1200_SOURCES
  CC1200.h
  CC1200.cpp
  CC1200Bits.h
  CC1200Morse.h
  CC1200Morse.cpp)

# Use these CMake options to turn debug prints on or off.
option(CC1200_DEBUG "If true, include debug strings into the code." TRUE)
option(CC1200_REGISTER_LEVEL_DEBUG "If true, print debug messages for individual register writes" FALSE)

add_library(cc1200 ${CC1200_SOURCES})
target_include_directories(cc1200 PUBLIC .)
target_link_libraries(cc1200 mbed-os)

# Prevent gcc complaining about unknown IDE pragmas
target_compile_options(cc1200 PRIVATE -Wno-unknown-pragmas)

# Apply debug definitions.
# These are only used in CC1200.cpp so can be set as PRIVATE.
if(CC1200_DEBUG)
    target_compile_definitions(cc1200 PRIVATE CC1200_DEBUG)
endif()
if(CC1200_REGISTER_LEVEL_DEBUG)
    target_compile_definitions(cc1200 PRIVATE CC1200_REGISTER_LEVEL_DEBUG)
endif()