set(CC1200_SOURCES
  CC1200.h
  CC1200.cpp
  CC1200Bits.h
  CC1200Morse.h
  CC1200Morse.cpp)

add_library(cc1200 ${CC1200_SOURCES})
target_include_directories(cc1200 PUBLIC .)
target_link_libraries(cc1200 mbed-os)