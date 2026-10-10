# ---------------------------------------------------------------------------
# flight-sim — the flight dynamics + telemetry simulation engine
#
# Layout:
#   include/fse/...   public headers, namespace `fse`
#   src/...           implementation
#   tests/...         GoogleTest based unit tests (see tests/CMakeLists.txt)
#
# The engine lives in the fse_core static library and the CLI is a thin
# executable on top of it. The library boundary is not cosmetic: it is what lets
# the tests exercise the engine at all, and what will let the protocol tooling
# link the codec without dragging in main().
# ---------------------------------------------------------------------------

add_library(fse_core STATIC
    src/atmosphere.cpp
    src/flight_model/aerodynamic_model.cpp
    src/flight_phase.cpp
    src/noise.cpp
)

add_library(fse::core ALIAS fse_core)

target_include_directories(fse_core
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

target_link_libraries(fse_core PUBLIC fse::compile_options)

add_executable(flight_sim
    src/main.cpp
)

target_link_libraries(flight_sim PRIVATE fse_core)

set_target_properties(flight_sim PROPERTIES
    OUTPUT_NAME "flight-sim"
    VS_DPI_AWARE "true"
)

# The wire protocol definition (protocol/telemetry_frame.hpp, milestone M2) is
# shared with the ground station and the golden-vector tooling. When it lands it
# will be exposed as a PUBLIC include directory here so tests and the code
# generator pick it up without re-declaring the path.

if(FSE_BUILD_TESTS)
    add_subdirectory(tests)
endif()