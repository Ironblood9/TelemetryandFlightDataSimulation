# ---------------------------------------------------------------------------
# Unit tests. GoogleTest is pulled in on demand so the repository stays free of
# third-party source; nothing is vendored and nothing has to be kept in sync.
# ---------------------------------------------------------------------------

include(FetchContent)

set(FSE_GOOGLETEST_TAG "v1.15.2" CACHE STRING "GoogleTest git tag to fetch")

FetchContent_Declare(
    googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG        ${FSE_GOOGLETEST_TAG}
    GIT_SHALLOW    TRUE
    # Treat the vendored sources as SYSTEM so our -Werror / -Wconversion policy
    # is not applied to third-party code.
    SYSTEM
)

# Must be set before MakeAvailable(): gtest must use the same CRT as the
# rest of the build on MSVC, otherwise every test fails to link.
set(gtest_force_shared_crt ON  CACHE BOOL "" FORCE)
set(INSTALL_GTEST        OFF CACHE BOOL "" FORCE)
set(BUILD_GMOCK          ON  CACHE BOOL "" FORCE)

FetchContent_MakeAvailable(googletest)

include(GoogleTest)

# --- fse_tests --------------------------------------------------------------
add_executable(fse_tests
    test_build_contract.cpp
    test_types.cpp
    test_noise.cpp
    test_phase_enum.cpp
)

target_link_libraries(fse_tests PRIVATE
    fse::compile_options
    fse::core
    GTest::gtest_main
)

target_include_directories(fse_tests PRIVATE
    "${CMAKE_SOURCE_DIR}/FlightSimulationEngine/include"
)

# NOTE: gtest_discover_tests() is deliberately NOT used. It runs the test
# executable at build time with `--gtest_output=json:<absolute path>`, and the
# MSVC CRT narrows `wargv` to `argv` using the ANSI code page. On a machine
# whose user name contains a character outside code page 1252 (for example the
# dotless i) the path arrives as "...\Ms?\..." and GoogleTest aborts with
# "Unable to open file". gtest_add_tests() scans the sources at configure time
# instead, so no runtime file I/O is involved and the suite is immune to it.
# See docs/architecture.md -> "Environment constraints".
gtest_add_tests(TARGET fse_tests
    PROPERTIES TIMEOUT 60
)