# ---------------------------------------------------------------------------
# flight-sim — the flight dynamics + telemetry simulation engine
#
# Layout (filled in as the milestones land):
#   include/fse/...   public headers, namespace `fse`
#   src/...           implementation
#   tests/...         GoogleTest based unit tests (see tests/CMakeLists.txt)
# ---------------------------------------------------------------------------

add_executable(flight_sim
    src/main.cpp
)

target_include_directories(flight_sim
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

target_link_libraries(flight_sim PRIVATE fse::compile_options)

# The wire protocol definition (protocol/telemetry_frame.hpp, milestone M2) is
# shared with the ground station and the golden-vector tooling. When it lands it
# will be exposed as a PUBLIC include directory here so tests and the code
# generator pick it up without re-declaring the path.

set_target_properties(flight_sim PROPERTIES
    OUTPUT_NAME "flight-sim"
    VS_DPI_AWARE "true"
)

if(FSE_BUILD_TESTS)
    add_subdirectory(tests)
endif()