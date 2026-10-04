# ---------------------------------------------------------------------------
# ProjectOptions.cmake
#
# Defines the single interface target `fse::compile_options` that carries
# warning, hardening and code-generation flags for every C++ target in the
# project. All per-configuration flags are expressed as generator expressions
# so one build tree can host several configurations at once.
# ---------------------------------------------------------------------------

option(FSE_BUILD_TESTS        "Build the C++ unit tests"                       ON)
option(FSE_ENABLE_SANITIZERS  "Enable AddressSanitizer / UndefinedBehaviorSanitizer" OFF)
option(FSE_WARNINGS_AS_ERRORS "Treat compiler warnings as errors"              OFF)
option(FSE_ENABLE_CLANG_TIDY  "Run clang-tidy as part of the build"             OFF)

if(TARGET fse::compile_options)
    return()
endif()

add_library(fse_compile_options INTERFACE)
add_library(fse::compile_options ALIAS fse_compile_options)

# --- Warnings ---------------------------------------------------------------
if(MSVC)
    target_compile_options(fse_compile_options INTERFACE
        /W4                     # high warning level
        /permissive-            # strict standard conformance
        /Zc:__cplusplus         # report the true __cplusplus value
        /Zc:preprocessor        # conformant preprocessor
        /utf-8                  # UTF-8 source and execution charset
        /MP                     # parallel compilation
        /diagnostics:caret
    )
    target_compile_definitions(fse_compile_options INTERFACE
        WIN32_LEAN_AND_MEAN
        NOMINMAX
        _CRT_SECURE_NO_WARNINGS
        UNICODE
        _UNICODE
    )
else()
    target_compile_options(fse_compile_options INTERFACE
        -Wall
        -Wextra
        -Wpedantic
        -Wshadow
        -Wnon-virtual-dtor
        -Wold-style-cast
        -Wcast-align
        -Wunused
        -Woverloaded-virtual
        # Numeric conversions are the #1 source of silent telemetry bugs
        # (millimetres vs metres, degrees vs radians, int16 vs float32).
        -Wconversion
        -Wsign-conversion
        -Wdouble-promotion
    )
endif()

if(FSE_WARNINGS_AS_ERRORS)
    if(MSVC)
        target_compile_options(fse_compile_options INTERFACE /WX)
    else()
        target_compile_options(fse_compile_options INTERFACE -Werror)
    endif()
endif()

# --- Sanitizers -------------------------------------------------------------
if(FSE_ENABLE_SANITIZERS)
    if(MSVC)
        # MSVC only ships the address sanitizer; undefined-behaviour sanitizing
        # is covered by the clang/gcc CI jobs instead.
        if(MSVC_VERSION GREATER_EQUAL 1929)
            target_compile_options(fse_compile_options INTERFACE /fsanitize=address /Zi)
            target_link_options(fse_compile_options INTERFACE /fsanitize=address)
        endif()
    else()
        target_compile_options(fse_compile_options INTERFACE
            -fsanitize=address,undefined
            -fno-sanitize-recover=all
            -fno-omit-frame-pointer
        )
        target_link_options(fse_compile_options INTERFACE -fsanitize=address,undefined)
    endif()
endif()

# --- Static analysis --------------------------------------------------------
if(FSE_ENABLE_CLANG_TIDY)
    find_program(CLANG_TIDY_EXE NAMES clang-tidy REQUIRED)
    # Set as a normal variable so every target defined *after* this point
    # inherits it.
    set(CMAKE_CXX_CLANG_TIDY "${CLANG_TIDY_EXE}")
    set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
endif()

# --- Build type specific optimisation ---------------------------------------
# Applied globally so Debug keeps frame pointers and Release gets full
# inlining without every target having to repeat itself.
if(MSVC)
    target_compile_options(fse_compile_options INTERFACE
        $<$<CONFIG:Debug>:/Od;/Zi;/RTC1>
        $<$<CONFIG:Release>:/O2;/GL>
        $<$<CONFIG:RelWithDebInfo>:/O2;/GL;/Zi>
    )
else()
    target_compile_options(fse_compile_options INTERFACE
        $<$<CONFIG:Debug>:-O0;-g3>
        $<$<CONFIG:Release>:-O3;-DNDEBUG>
        $<$<CONFIG:RelWithDebInfo>:-O2;-g;-DNDEBUG>
    )
endif()