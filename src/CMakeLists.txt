rock_library(kdl_conversions
    SOURCES Dummy.cpp
    HEADERS Dummy.hpp)

rock_executable(kdl_conversions_bin Main.cpp
    DEPS kdl_conversions)

