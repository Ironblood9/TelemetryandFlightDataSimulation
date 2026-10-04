// ---------------------------------------------------------------------------
// flight-sim — entry point.
//
// Milestone M0 only wires up the build system and the toolchain contract.
// The flight dynamics model, the sensor suite and the telemetry transport
// land in the following milestones; see docs/architecture.md for the roadmap.
// ---------------------------------------------------------------------------

#include <cstdlib>
#include <cstring>
#include <iostream>
#include <string_view>

#include "fse/version.hpp"

namespace
{

constexpr std::string_view kUsage = R"(flight-sim — flight dynamics & telemetry simulation engine

Usage:
  flight-sim [options]

Options:
  -h, --help       Show this message and exit.
  -v, --version    Print version information and exit.

Milestone M0 only provides the toolchain skeleton; the simulation loop,
the sensor suite and the UDP transport arrive in M1-M3.
)";

void print_version()
{
    std::cout << fse::version::kApplicationName << ' ' << fse::version::kString << '\n'
              << "telemetry protocol version : " << static_cast<int>(fse::kProtocolVersion) << '\n'
              << "default sample rate        : " << fse::kDefaultSampleRateHz << " Hz\n";
}

enum class ParseResult
{
    kRun,         // no arguments: start the simulation
    kExitSuccess, // --help / --version already produced their output
    kExitFailure, // the command line was rejected
};

ParseResult parse_command_line(int argc, char** argv)
{
    bool has_errors = false;

    for (int i = 1; i < argc; ++i)
    {
        const std::string_view arg{argv[i]};

        if (arg == "-h" || arg == "--help")
        {
            std::cout << kUsage;
            return ParseResult::kExitSuccess;
        }
        if (arg == "-v" || arg == "--version")
        {
            print_version();
            return ParseResult::kExitSuccess;
        }

        // Report every bad argument instead of bailing out on the first one.
        std::cerr << "error: unknown argument '" << arg << "'\n";
        has_errors = true;
    }

    if (has_errors)
    {
        std::cerr << '\n' << kUsage;
        return ParseResult::kExitFailure;
    }

    return ParseResult::kRun;
}

} // namespace

int main(int argc, char** argv)
{
    const ParseResult result = parse_command_line(argc, argv);

    if (result == ParseResult::kExitSuccess)
    {
        return EXIT_SUCCESS;
    }
    if (result == ParseResult::kExitFailure)
    {
        return EXIT_FAILURE;
    }

    print_version();
    std::cout << '\n' << kUsage;
    return EXIT_SUCCESS;
}