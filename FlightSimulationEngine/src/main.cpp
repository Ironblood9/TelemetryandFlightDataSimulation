// ---------------------------------------------------------------------------
// flight-sim — entry point.
//
// Runs the flight simulation headless and writes a telemetry trace. The
// simulation itself is deterministic for a given seed, so this program is also
// the regression harness: a recorded CSV from a given seed must be byte
// identical on every run.
// ---------------------------------------------------------------------------

#include "fse/simulation.hpp"
#include "fse/version.hpp"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <optional>
#include <string>
#include <string_view>

namespace
{

/// Upper bounds accepted on the command line. They exist so that the step count
/// can be computed and converted to an integer without overflowing: a floating
/// point value outside the range of the target integer type is undefined
/// behaviour, not a saturated one, so an unchecked conversion here would only
/// show up as a sanitizer failure.
constexpr double kMaxDurationS = 86400.0; // one day of simulated time
constexpr double kMaxRateHz    = 1000.0;

constexpr std::string_view kUsage = R"(flight-sim — flight dynamics & telemetry simulation engine

Usage:
  flight-sim [options]

Options:
  --rate <hz>          Telemetry production rate          [1..1000]  (default 100)
  --duration <s>       Simulated seconds to run                     (default 120)
  --seed <n>           Seed for every noise generator               (default 0)
  --csv <path>         Write a telemetry trace to a relative CSV path
  --phase <name>       Start in a phase          (preflight|taxi|takeoff_roll|climb|
                                                cruise|descent|approach|landing|
                                                go_around|failure)
  --fault <name>       Latch a fault at startup    (engine_fire|engine_out|sensor_fault|
                                                hydraulic_low|degraded_airframe)
  --no-wind            Disable steady wind and gusts
  --quiet              Do not print the phase trace
  -h, --help           Show this message
  -v, --version        Show version information

Examples:
  flight-sim --duration 240 --csv out/flight.csv
  flight-sim --phase cruise --rate 200 --seed 7
)";

struct Options
{
    std::uint16_t    rate_hz{100};
    double           duration_s{120.0};
    std::uint64_t    seed{0};
    std::string      csv_path;
    fse::FlightPhase initial_phase{fse::FlightPhase::kPreflight};
    fse::FaultCode   fault{fse::FaultCode::kNone};
    bool             wind_enabled{true};
    bool             quiet{false};
};

void print_version()
{
    std::cout << fse::version::kApplicationName << ' ' << fse::version::kString << '\n'
              << "telemetry protocol version : " << static_cast<int>(fse::kProtocolVersion) << '\n'
              << "default sample rate        : " << fse::kDefaultSampleRateHz << " Hz\n";
}

/// Parses a non negative integer, rejecting trailing garbage so that `--seed 1x`
/// is an error rather than silently becoming 1.
[[nodiscard]] std::optional<std::uint64_t> parse_u64(std::string_view text)
{
    if (text.empty())
    {
        return std::nullopt;
    }

    std::uint64_t value = 0U;
    for (const char c : text)
    {
        if (c < '0' || c > '9')
        {
            return std::nullopt;
        }
        const auto digit = static_cast<std::uint64_t>(c - '0');
        if (value > (UINT64_MAX - digit) / 10U)
        {
            return std::nullopt;
        }
        value = (value * 10U) + digit;
    }
    return value;
}

[[nodiscard]] std::optional<double> parse_double(std::string_view text)
{
    if (text.empty())
    {
        return std::nullopt;
    }
    try
    {
        std::size_t       consumed = 0U;
        const std::string owned(text);
        const double      value = std::stod(owned, &consumed);
        if (consumed != owned.size() || !std::isfinite(value))
        {
            return std::nullopt;
        }
        return value;
    }
    catch (...)
    {
        return std::nullopt;
    }
}

[[nodiscard]] std::optional<fse::FaultCode> parse_fault(std::string_view name)
{
    if (name == "engine_fire")
    {
        return fse::FaultCode::kEngineFire;
    }
    if (name == "engine_out")
    {
        return fse::FaultCode::kEngineOut;
    }
    if (name == "sensor_fault")
    {
        return fse::FaultCode::kSensorFault;
    }
    if (name == "hydraulic_low")
    {
        return fse::FaultCode::kHydraulicLow;
    }
    if (name == "degraded_airframe")
    {
        return fse::FaultCode::kDegradedAirframe;
    }
    return std::nullopt;
}

enum class ParseOutcome
{
    kRun,
    kExitSuccess,
    kExitFailure,
};

ParseOutcome parse_command_line(int argc, char** argv, Options& options)
{
    bool has_errors = false;

    const auto fail = [&has_errors](std::string_view message, std::string_view argument)
    {
        std::cerr << "error: " << message << " '" << argument << "'\n";
        has_errors = true;
    };

    // `needs_value` returns the argument after a flag, or reports the error.
    const auto needs_value = [&](int& index, std::string_view flag) -> std::string_view
    {
        if ((index + 1) >= argc)
        {
            fail("missing value for", flag);
            return {};
        }
        ++index;
        return std::string_view(argv[index]);
    };

    for (int i = 1; i < argc; ++i)
    {
        const std::string_view arg{argv[i]};

        if (arg == "-h" || arg == "--help")
        {
            std::cout << kUsage;
            return ParseOutcome::kExitSuccess;
        }
        if (arg == "-v" || arg == "--version")
        {
            print_version();
            return ParseOutcome::kExitSuccess;
        }
        if (arg == "--quiet")
        {
            options.quiet = true;
            continue;
        }
        if (arg == "--no-wind")
        {
            options.wind_enabled = false;
            continue;
        }

        if (arg == "--rate")
        {
            const std::string_view value = needs_value(i, arg);
            if (const auto parsed = parse_u64(value))
            {
                if (*parsed < 1U || *parsed > 1000U)
                {
                    fail("rate must be between 1 and 1000 Hz", value);
                }
                else
                {
                    options.rate_hz = static_cast<std::uint16_t>(*parsed);
                }
            }
            else
            {
                fail("invalid rate", value);
            }
            continue;
        }

        if (arg == "--duration")
        {
            const std::string_view value = needs_value(i, arg);
            if (const auto parsed = parse_double(value))
            {
                if (*parsed < 0.0)
                {
                    fail("duration must not be negative", value);
                }
                else
                {
                    options.duration_s = *parsed;
                }
            }
            else
            {
                fail("invalid duration", value);
            }
            continue;
        }

        if (arg == "--seed")
        {
            const std::string_view value = needs_value(i, arg);
            if (const auto parsed = parse_u64(value))
            {
                options.seed = *parsed;
            }
            else
            {
                fail("invalid seed", value);
            }
            continue;
        }

        if (arg == "--csv")
        {
            const std::string_view value = needs_value(i, arg);
            if (value.empty())
            {
                fail("empty csv path", arg);
            }
            else
            {
                options.csv_path = std::string(value);
            }
            continue;
        }

        if (arg == "--phase")
        {
            const std::string_view value = needs_value(i, arg);
            if (const auto parsed = fse::flight_phase_from_string(value))
            {
                options.initial_phase = *parsed;
            }
            else
            {
                fail("unknown phase", value);
            }
            continue;
        }

        if (arg == "--fault")
        {
            const std::string_view value = needs_value(i, arg);
            if (const auto parsed = parse_fault(value))
            {
                options.fault = *parsed;
            }
            else
            {
                fail("unknown fault", value);
            }
            continue;
        }

        fail("unknown argument", arg);
    }

    if (has_errors)
    {
        std::cerr << '\n' << kUsage;
        return ParseOutcome::kExitFailure;
    }

    return ParseOutcome::kRun;
}

/// Writes the trace as CSV.
///
/// The header order matches the telemetry frame in docs/protocol-spec.md, so a
/// diff of two runs is readable line by line.
class CsvWriter
{
public:
    /// Number of comma-separated fields the header declares.
    ///
    /// A data row whose field count does not match the header is worse than no
    /// file at all: every column is then silently attributed to the wrong name.
    /// Rather than trust that by inspection, the writer counts the fields it
    /// emits and refuses to continue if the two ever disagree.
    static constexpr std::size_t kColumnCount = 20;

    [[nodiscard]] bool open(const std::string& path)
    {
        stream_.open(path, std::ios::out | std::ios::trunc);
        if (!stream_.is_open())
        {
            return false;
        }
        stream_ << "seq,timestamp_ns,phase,status_bits,faults,airspeed_mps,altitude_m,"
                   "vertical_speed_mps,heading_deg,pitch_deg,roll_deg,"
                   "engine_temp_0_c,engine_temp_1_c,engine_temp_2_c,engine_temp_3_c,"
                   "fuel_press_kpa,oil_press_kpa,accel_x_mps2,accel_y_mps2,accel_z_mps2\n";
        return true;
    }

    void write(const fse::SensorSample& sample)
    {
        // Five fixed-width leading fields, then one per write_scalar() call.
        // kLeadingFields + the write_scalar() calls must add up to kColumnCount.
        fields_written_ = kLeadingFields;

        // No trailing comma here: write_scalar() emits its own leading comma, so
        // writing one here as well would open an empty extra field and shift
        // every column after `faults` one position to the right of the header.
        stream_ << sample.sequence << ',' << sample.timestamp_ns << ','
                << fse::to_string(sample.phase) << ',' << sample.status_bits << ','
                << fse::to_bits(sample.faults);

        write_scalar(sample.airspeed_mps);
        write_scalar(sample.altitude_m);
        write_scalar(sample.vertical_speed_mps);
        write_scalar(sample.heading_deg);
        write_scalar(sample.pitch_deg);
        write_scalar(sample.roll_deg);

        for (const fse::Scalar temperature : sample.engine_temp_c)
        {
            write_scalar(temperature);
        }

        write_scalar(sample.fuel_press_kpa);
        write_scalar(sample.oil_press_kpa);
        write_scalar(sample.accel_mps2.x);
        write_scalar(sample.accel_mps2.y);
        write_scalar(sample.accel_mps2.z);

        if (fields_written_ != kColumnCount)
        {
            std::cerr << "internal error: wrote " << fields_written_
                      << " CSV fields, header declares " << kColumnCount << '\n';
            std::exit(EXIT_FAILURE);
        }

        stream_ << '\n';
    }

    [[nodiscard]] bool is_open() const
    {
        return stream_.is_open();
    }

    /// Only meaningful once the file has been opened: `good()` on a stream that
    /// was never opened is false, which would otherwise report failure for every
    /// run that was not asked to write a trace.
    [[nodiscard]] bool ok() const
    {
        return stream_.good();
    }

private:
    /// seq, timestamp_ns, phase, status_bits, faults
    static constexpr std::size_t kLeadingFields = 5;

    void write_scalar(fse::Scalar value)
    {
        stream_ << ',' << std::fixed << std::setprecision(4) << value;
        ++fields_written_;
    }

    std::ofstream stream_;
    std::size_t   fields_written_{0};
};

} // namespace

int main(int argc, char** argv)
{
    Options            options;
    const ParseOutcome outcome = parse_command_line(argc, argv, options);
    if (outcome != ParseOutcome::kRun)
    {
        return (outcome == ParseOutcome::kExitSuccess) ? EXIT_SUCCESS : EXIT_FAILURE;
    }

    fse::SimulationConfig config;
    config.rate_hz       = options.rate_hz;
    config.seed          = options.seed;
    config.initial_phase = options.initial_phase;
    config.wind_enabled  = options.wind_enabled;

    fse::Simulation simulation{config};

    if (options.fault != fse::FaultCode::kNone)
    {
        simulation.latch_fault(options.fault);
    }

    CsvWriter csv;
    if (!options.csv_path.empty())
    {
        if (!csv.open(options.csv_path))
        {
            std::cerr << "error: cannot open '" << options.csv_path << "' for writing\n";
            return EXIT_FAILURE;
        }
        if (!options.quiet)
        {
            // Native tools must use relative paths here: the MSVC CRT narrows
            // argv through the ANSI code page, and a user name outside code page
            // 1252 turns an absolute path into an invalid one.
            std::cerr << "note: writing the trace to the relative path '" << options.csv_path
                      << "'\n";
        }
    }

    if (!options.quiet)
    {
        std::cerr << "phase trace (seed " << options.seed << ", " << options.rate_hz << " Hz)\n";
    }

    // Computed in double and range checked before the narrowing conversion.
    // Converting an out-of-range floating point value to an integer type is
    // undefined behaviour, not a saturated result, so a generous --duration
    // would be caught by the sanitizer job rather than clamped here.
    const double requested_steps = options.duration_s * static_cast<double>(options.rate_hz);
    const double max_steps = static_cast<double>(kMaxDurationS) * static_cast<double>(kMaxRateHz);

    if (requested_steps > max_steps)
    {
        std::cerr << "error: --duration " << options.duration_s << " s at " << options.rate_hz
                  << " Hz is out of range (maximum " << kMaxDurationS << " s)\n";
        return EXIT_FAILURE;
    }

    const auto total_steps = static_cast<std::uint64_t>(requested_steps);

    std::size_t reported_changes = 0U;
    for (std::uint64_t i = 0; i < total_steps; ++i)
    {
        const fse::SensorSample& sample = simulation.step();

        if (csv.is_open())
        {
            csv.write(sample);
        }

        if (!options.quiet)
        {
            const auto& changes = simulation.phase_changes();
            while (reported_changes < changes.size())
            {
                const fse::PhaseChange& change = changes[reported_changes];
                std::cerr << "  [" << std::setw(8) << std::fixed << std::setprecision(2)
                          << static_cast<double>(change.step_index) /
                                 static_cast<double>(options.rate_hz)
                          << " s] " << std::setw(7) << std::setfill('0') << std::setw(6)
                          << change.sequence << "  " << fse::to_string(change.from) << " -> "
                          << fse::to_string(change.to) << "  (" << change.reason << ")\n"
                          << std::setfill(' ');
                ++reported_changes;
            }
        }
    }

    const fse::AircraftState& state = simulation.aircraft_state();

    if (!options.quiet)
    {
        std::cerr << "final state\n"
                  << "  phase        : " << fse::to_string(state.phase) << '\n'
                  << "  altitude     : " << std::fixed << std::setprecision(1)
                  << static_cast<double>(state.position_m.y) << " m\n"
                  << "  ias          : " << static_cast<double>(state.ias_mps) << " m/s\n"
                  << "  heading      : " << static_cast<double>(state.heading_deg) << " deg\n"
                  << "  fuel         : " << static_cast<double>(state.fuel_kg) << " kg\n"
                  << "  transitions  : " << simulation.phase_changes().size() << '\n'
                  << "  samples      : " << simulation.step_index() << '\n';
    }

    return (!csv.is_open() || csv.ok()) ? EXIT_SUCCESS : EXIT_FAILURE;
}
