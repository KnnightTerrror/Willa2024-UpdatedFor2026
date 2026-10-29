#include "infrastructure/SparkMax.h"

#include <frc/DataLogManager.h>
#include <frc/RobotController.h>
#include <frc/shuffleboard/BuiltInWidgets.h>
#include <frc/shuffleboard/SimpleWidget.h>
#include <units/voltage.h>
#include <wpi/DataLog.h>

#include <rev/REVLibError.h>
#include <rev/SparkBase.h>
#include <rev/SparkFlex.h>
#include <rev/SparkMax.h>
#include <rev/config/SparkBaseConfig.h>
#include <rev/config/SparkFlexConfig.h>
#include <rev/config/SparkMaxConfig.h>

#include <cstdint>
#include <cstdio>
#include <exception>
#include <functional>
#include <memory>
#include <string>
#include <string_view>
#include <variant>

using namespace rev::spark;

// SPARK MAX/FLEX is a smart motor controller, which communicates via the CAN
// bus.  There are periodic packets which flow both to and from this
// controller.  Getting position, velocity, faults, etc. simply reports the
// most recently received periodic status, so these calls are fast and
// essentially free.  Configure() and ClearFaults() require round-trip
// messaging and are slow, so they are only done when explicitly requested
// (or following a controller reset).  One of the main points of the
// SmartMotor and SparkMax classes is to encapsulate this complexity, including
// robust error handling, offering an API which is much easier to use
// correctly.  This code will run without any motor controllers present.

namespace
{
    class SparkMotor : public SmartMotorBase
    {
    public:
        SparkMotor(const std::string_view name, const bool flex_not_max, const int canId, const bool inverted) noexcept;

        SparkMotor(const SparkMotor &) = delete;
        SparkMotor &operator=(const SparkMotor &) = delete;

        const std::string &GetName() const noexcept override;

        void SetConfig(const ConfigMap config) noexcept override;

        void AddConfig(const ConfigMap config) noexcept override;

        void CheckConfig() noexcept override;

        void ApplyConfig(bool burn) noexcept override;

        void ClearFaults() noexcept override;

        bool GetStatus() noexcept override;

        void Periodic() noexcept override;

        void ShuffleboardCreate(frc::ShuffleboardContainer &container,
                                std::function<void(double)> control = nullptr,
                                std::function<void()> reset = nullptr) noexcept override;

        void SetIdleMode(const IdleMode mode) noexcept override;

        IdleMode GetIdleMode() noexcept override;

        void EnableLimit(const Direction direction) noexcept override;

        void DisableLimit(const Direction direction) noexcept override;

        bool GetLimit(const Direction direction) noexcept override;

        void Stop() noexcept override;

        void Set(const double percent) noexcept override;

        double Get() noexcept override;

        void SetVoltage(const units::volt_t voltage) noexcept override;

        void SetCurrent(const units::ampere_t current) noexcept override;

        units::volt_t GetVoltage() noexcept override;

        units::ampere_t GetCurrent() noexcept override;

        void SpecifyPosition(const double position) noexcept override;

        void SeekPosition(const double position) noexcept override;

        bool CheckPosition(const double tolerance) noexcept override;

        double GetPositionRaw() noexcept override;

        void SeekVelocity(const double velocity) noexcept override;

        bool CheckVelocity(const double tolerance) noexcept override;

        double GetVelocityRaw() noexcept override;

    private:
        void ShuffleboardPeriodic() noexcept;
        void DoSafely(const char *const what, std::function<void()> work) noexcept;
        bool AnyError(const rev::REVLibError returnCode) noexcept;
        std::string BuildConfig(SparkBaseConfig &sparkConfig) noexcept;
        void Configure(const bool burn) noexcept;

        // Parameters supplied though ctor.
        const std::string name_;
        const bool flex_not_max_;
        const int canId_;
        const bool inverted_;

        // Used for printf-style logging.
        wpi::log::StringLogEntry m_stringLog;

        // Underlying REV object holders.  Only one of these is used, depending
        // on the type of motor controller.  A pointer to the common base class
        // is used wherever possible, so that the code is identical for either.
        std::unique_ptr<rev::spark::SparkMax> max_motor_;
        std::unique_ptr<rev::spark::SparkFlex> flex_motor_;
        SparkBase *motor_{nullptr};

        // Shuffleboard UI elements, used by ShuffleboardCreate() and
        // ShuffleboardPeriodic() only.
        bool shuffleboard_{false};
        frc::SimpleWidget *temperatureUI_{nullptr};
        frc::SimpleWidget *statusUI_{nullptr};
        frc::SimpleWidget *faultsUI_{nullptr};
        frc::SimpleWidget *stickyFaultsUI_{nullptr};
        frc::SimpleWidget *voltageUI_{nullptr};
        frc::SimpleWidget *currentUI_{nullptr};
        frc::SimpleWidget *percentUI_{nullptr};
        frc::SimpleWidget *speedUI_{nullptr};
        frc::SimpleWidget *distanceUI_{nullptr};
        frc::SimpleWidget *velocityUI_{nullptr};
        frc::SimpleWidget *controlUI_{nullptr};
        frc::SimpleWidget *resetUI_{nullptr};

        // Shuffleboard UI control element mechanism.
        double previousControl_{0.0};
        std::function<void(double)> controlF_{nullptr};
        std::function<void()> resetF_{nullptr};

        // Configuration Parameters, set via SetConfig() and AddConfig().
        ConfigMap config_;
        std::string configReporting_;

        // The next two data members are used to spread out any work done in
        // Periodic(), so that all motor controllers do not do this work
        // during the same iteration.
        uint64_t FPGATime_{frc::RobotController::GetFPGATime()};
        int iteration_{0};

        // Current configuration verification status.
        bool configGood_{false};

        // Config has been saved to flash (at least once since startup).
        bool persisted_{false};

        // Current idle mode (there is no cheap way to read this back).
        IdleMode idleMode_{IdleMode::kBrake};

        // Limit switch enables (applied via config).
        bool forwardLimitEnabled_{true};
        bool reverseLimitEnabled_{true};

        // Last commanded position or velocity.
        double position_{0.0};
        double velocity_{0.0};

        // Cumulative faults/warnings (since last explicit clear).
        uint16_t faultBits_{0};
        uint16_t warningBits_{0};

        // Counters for all types of error (since last explicit clear).
        uint resets_{0};
        uint catches_{0};
        uint errors_{0};
        int lastError_{0};
    };

    // SPARK Fault Codes with shorthand characters used here:
    // O other
    // M motorType
    // S sensor
    // C can
    // T temperature
    // G gateDriver
    // E escEeprom
    // F firmware
    std::string FaultInfo(const SparkBase::Faults &faults) noexcept
    {
        std::string faultInfo;

        faultInfo += faults.other ? 'O' : '.';
        faultInfo += faults.motorType ? 'M' : '.';
        faultInfo += faults.sensor ? 'S' : '.';
        faultInfo += faults.can ? 'C' : '.';
        faultInfo += faults.temperature ? 'T' : '.';
        faultInfo += faults.gateDriver ? 'G' : '.';
        faultInfo += faults.escEeprom ? 'E' : '.';
        faultInfo += faults.firmware ? 'F' : '.';

        return faultInfo;
    }

    // SPARK Warning Codes with shorthand characters used here:
    // B brownout
    // C overcurrent
    // E escEeprom
    // X extEeprom
    // S sensor
    // L stall
    // R hasReset
    // O other
    std::string WarningInfo(const SparkBase::Warnings &warnings) noexcept
    {
        std::string warningInfo;

        warningInfo += warnings.brownout ? 'B' : '.';
        warningInfo += warnings.overcurrent ? 'C' : '.';
        warningInfo += warnings.escEeprom ? 'E' : '.';
        warningInfo += warnings.extEeprom ? 'X' : '.';
        warningInfo += warnings.sensor ? 'S' : '.';
        warningInfo += warnings.stall ? 'L' : '.';
        warningInfo += warnings.hasReset ? 'R' : '.';
        warningInfo += warnings.other ? 'O' : '.';

        return warningInfo;
    }

    std::string ValueInfo(const SmartMotorBase::ConfigValue &value) noexcept
    {
        if (const bool *const pbool = std::get_if<bool>(&value))
        {
            return *pbool ? "true" : "false";
        }
        if (const uint *const puint = std::get_if<uint>(&value))
        {
            return std::to_string(*puint);
        }
        if (const double *const pdouble = std::get_if<double>(&value))
        {
            return std::to_string(*pdouble);
        }

        return "?";
    }
}

void SparkMaxFactory::ConfigIndex() noexcept
{
    for (const auto &elem : configDefaults)
    {
        std::printf("Config: %s = %s\n", elem.first.c_str(), ValueInfo(elem.second).c_str());
    }
}

std::unique_ptr<SmartMotorBase> SparkMaxFactory::CreateSparkMax(const std::string_view name, const int canId, const bool inverted) noexcept
{
    return std::make_unique<SparkMotor>(name, false, canId, inverted);
}

std::unique_ptr<SmartMotorBase> SparkMaxFactory::CreateSparkFlex(const std::string_view name, const int canId, const bool inverted) noexcept
{
    return std::make_unique<SparkMotor>(name, true, canId, inverted);
}

SparkMotor::SparkMotor(const std::string_view name, const bool flex_not_max, const int canId, const bool inverted) noexcept : name_{name}, flex_not_max_{flex_not_max}, canId_{canId}, inverted_{inverted}
{
    // Set up onboard printf-style logging.
    std::string logName{"/SparkMax/"};
    logName += name;
    logName += '[';
    logName += std::to_string(canId);
    logName += ']';
    m_stringLog = wpi::log::StringLogEntry(frc::DataLogManager::GetLog(), logName);

    DoSafely("ctor", [&]() -> void
             {
        if (flex_not_max_)
        {
            flex_motor_ = std::make_unique<rev::spark::SparkFlex>(canId_, SparkLowLevel::MotorType::kBrushless);
            motor_ = flex_motor_.get();
        }
        else
        {
            max_motor_ = std::make_unique<rev::spark::SparkMax>(canId_, SparkLowLevel::MotorType::kBrushless);
            motor_ = max_motor_.get();
        } });

    SetConfig(ConfigMap());
}

const std::string &SparkMotor::GetName() const noexcept
{
    return name_;
}

void SparkMotor::SetConfig(const ConfigMap config) noexcept
{
    config_ = config;

    // Ensure at least this config parameter is managed.
    if (config_.count("kIdleMode") == 0)
    {
        config_["kIdleMode"] = std::get<uint>(SparkMaxFactory::configDefaults.at("kIdleMode"));
    }

    if (const uint *const puint = std::get_if<uint>(&config_.at("kIdleMode")))
    {
        idleMode_ = *puint == 0 ? IdleMode::kCoast : IdleMode::kBrake;
    }
}

void SparkMotor::AddConfig(const ConfigMap config) noexcept
{
    // New values replace any existing values.
    for (const auto &elem : config)
    {
        config_[elem.first] = elem.second;
    }

    const auto it = config_.find("kIdleMode");

    if (it != config_.end())
    {
        if (const uint *const puint = std::get_if<uint>(&it->second))
        {
            idleMode_ = *puint == 0 ? IdleMode::kCoast : IdleMode::kBrake;
        }
    }
}

// With REVLib 2026, configuration is applied as a whole, and there is no
// longer an efficient way to read back each parameter.  So, this reports on
// the last application of config parameters and any problems with the keys
// or values that were specified.
void SparkMotor::CheckConfig() noexcept
{
    SparkBaseConfig *sparkConfig{nullptr};
    SparkMaxConfig maxConfig;
    SparkFlexConfig flexConfig;

    if (flex_not_max_)
    {
        sparkConfig = &flexConfig;
    }
    else
    {
        sparkConfig = &maxConfig;
    }

    const std::string reporting = BuildConfig(*sparkConfig);

    if (reporting != configReporting_)
    {
        configReporting_ = reporting;

        if (configReporting_.empty())
        {
            std::printf("SparkMax[%i] %s config: OK.\n", canId_, name_.c_str());
        }
        else
        {
            std::printf("SparkMax[%i] %s config: %s.\n", canId_, name_.c_str(), configReporting_.c_str());
        }
    }
}

void SparkMotor::ApplyConfig(bool burn) noexcept
{
    Configure(burn);
}

// Translate `config_` into a REVLib config object.  Returns a string reporting
// any problems (empty if there were none).
std::string SparkMotor::BuildConfig(SparkBaseConfig &sparkConfig) noexcept
{
    std::string reporting;

    auto report = [&](const std::string &key, const char *const problem) -> void
    {
        if (!reporting.empty())
        {
            reporting += " ";
        }

        reporting += key;
        reporting += "(";
        reporting += problem;
        reporting += ")";
    };

    // Obtain a config value, of the expected type (using the defaults table).
    // Returns the default if not specified (or if there is a type mismatch).
    auto getBool = [&](const std::string &key) -> bool
    {
        const auto it = config_.find(key);

        if (it != config_.end())
        {
            if (const bool *const p = std::get_if<bool>(&it->second))
            {
                return *p;
            }
        }

        return std::get<bool>(SparkMaxFactory::configDefaults.at(key));
    };

    auto getUint = [&](const std::string &key) -> uint
    {
        const auto it = config_.find(key);

        if (it != config_.end())
        {
            if (const uint *const p = std::get_if<uint>(&it->second))
            {
                return *p;
            }
        }

        return std::get<uint>(SparkMaxFactory::configDefaults.at(key));
    };

    auto getDouble = [&](const std::string &key) -> double
    {
        const auto it = config_.find(key);

        if (it != config_.end())
        {
            if (const double *const p = std::get_if<double>(&it->second))
            {
                return *p;
            }
        }

        return std::get<double>(SparkMaxFactory::configDefaults.at(key));
    };

    // First, validate all the specified keys and the types of the values.
    for (const auto &elem : config_)
    {
        const auto kv = SparkMaxFactory::configDefaults.find(elem.first);

        if (kv == SparkMaxFactory::configDefaults.end())
        {
            report(elem.first, "Invalid");
        }
        else if (kv->second.index() != elem.second.index())
        {
            report(elem.first, "Type");
        }
    }

    // Inversion is handled through the constructor; always apply it.
    sparkConfig.Inverted(inverted_);

    sparkConfig.SetIdleMode(idleMode_ == IdleMode::kBrake ? SparkBaseConfig::IdleMode::kBrake : SparkBaseConfig::IdleMode::kCoast);

    // Status frame periods; these are now part of the config ("signals").
    if (config_.count("kStatus0") != 0)
    {
        const int period = static_cast<int>(getUint("kStatus0"));

        sparkConfig.signals.AppliedOutputPeriodMs(period);
        sparkConfig.signals.FaultsPeriodMs(period);
        sparkConfig.signals.WarningsPeriodMs(period);
    }

    if (config_.count("kStatus1") != 0)
    {
        const int period = static_cast<int>(getUint("kStatus1"));

        sparkConfig.signals.PrimaryEncoderVelocityPeriodMs(period);
        sparkConfig.signals.BusVoltagePeriodMs(period);
        sparkConfig.signals.OutputCurrentPeriodMs(period);
        sparkConfig.signals.MotorTemperaturePeriodMs(period);
    }

    if (config_.count("kStatus2") != 0)
    {
        sparkConfig.signals.PrimaryEncoderPositionPeriodMs(static_cast<int>(getUint("kStatus2")));
    }

    // Limit switches.
    if (config_.count("kLimitSwitchFwdPolarity") != 0)
    {
        sparkConfig.limitSwitch.ForwardLimitSwitchType(getUint("kLimitSwitchFwdPolarity") == 0 ? LimitSwitchConfig::Type::kNormallyOpen : LimitSwitchConfig::Type::kNormallyClosed);
    }

    if (config_.count("kLimitSwitchRevPolarity") != 0)
    {
        sparkConfig.limitSwitch.ReverseLimitSwitchType(getUint("kLimitSwitchRevPolarity") == 0 ? LimitSwitchConfig::Type::kNormallyOpen : LimitSwitchConfig::Type::kNormallyClosed);
    }

    if (config_.count("kHardLimitFwdEn") != 0)
    {
        forwardLimitEnabled_ = getBool("kHardLimitFwdEn");
    }

    if (config_.count("kHardLimitRevEn") != 0)
    {
        reverseLimitEnabled_ = getBool("kHardLimitRevEn");
    }

    sparkConfig.limitSwitch.ForwardLimitSwitchTriggerBehavior(forwardLimitEnabled_ ? LimitSwitchConfig::Behavior::kStopMovingMotor : LimitSwitchConfig::Behavior::kKeepMovingMotor);
    sparkConfig.limitSwitch.ReverseLimitSwitchTriggerBehavior(reverseLimitEnabled_ ? LimitSwitchConfig::Behavior::kStopMovingMotor : LimitSwitchConfig::Behavior::kKeepMovingMotor);

    // Follower mode; kFollowerID holds the CAN ID of the leader in the six
    // least-significant bits, and bit 0x00004000 of kFollowerConfig specifies
    // inversion (this is how things were encoded with the 2024 REVLib).
    if (config_.count("kFollowerID") != 0 || config_.count("kFollowerConfig") != 0)
    {
        const uint followerID = getUint("kFollowerID");
        const uint followerConfig = getUint("kFollowerConfig");

        if (followerID == 0)
        {
            sparkConfig.DisableFollowerMode();
        }
        else
        {
            sparkConfig.Follow(static_cast<int>(followerID & 0x3f), (followerConfig & 0x00004000) != 0);
        }
    }

    // Soft limits.
    if (config_.count("kSoftLimitFwd") != 0)
    {
        sparkConfig.softLimit.ForwardSoftLimit(getDouble("kSoftLimitFwd"));
    }

    if (config_.count("kSoftLimitRev") != 0)
    {
        sparkConfig.softLimit.ReverseSoftLimit(getDouble("kSoftLimitRev"));
    }

    // Ramp rates.
    if (config_.count("kRampRate") != 0)
    {
        sparkConfig.OpenLoopRampRate(getDouble("kRampRate"));
    }

    if (config_.count("kClosedLoopRampRate") != 0)
    {
        sparkConfig.ClosedLoopRampRate(getDouble("kClosedLoopRampRate"));
    }

    // Voltage compensation.
    if (config_.count("kCompensatedNominalVoltage") != 0)
    {
        const double nominalVoltage = getDouble("kCompensatedNominalVoltage");

        if (nominalVoltage > 0.0)
        {
            sparkConfig.VoltageCompensation(nominalVoltage);
        }
        else
        {
            sparkConfig.DisableVoltageCompensation();
        }
    }

    // Built-in (hall sensor) encoder.
    if (config_.count("kEncoderAverageDepth") != 0)
    {
        sparkConfig.encoder.UvwAverageDepth(static_cast<int>(getUint("kEncoderAverageDepth")));
    }

    if (config_.count("kEncoderSampleDelta") != 0)
    {
        sparkConfig.encoder.UvwMeasurementPeriod(static_cast<int>(getUint("kEncoderSampleDelta")));
    }

    if (config_.count("kPositionConversionFactor") != 0)
    {
        sparkConfig.encoder.PositionConversionFactor(getDouble("kPositionConversionFactor"));
    }

    if (config_.count("kVelocityConversionFactor") != 0)
    {
        sparkConfig.encoder.VelocityConversionFactor(getDouble("kVelocityConversionFactor"));
    }

    // Current limits.
    if (config_.count("kCurrentChop") != 0 || config_.count("kCurrentChopCycles") != 0)
    {
        const double currentChop = getDouble("kCurrentChop");

        if (currentChop > 0.0)
        {
            sparkConfig.SecondaryCurrentLimit(currentChop, static_cast<int>(getUint("kCurrentChopCycles")));
        }
    }

    if (config_.count("kSmartCurrentStallLimit") != 0 ||
        config_.count("kSmartCurrentFreeLimit") != 0 ||
        config_.count("kSmartCurrentConfig") != 0)
    {
        sparkConfig.SmartCurrentLimit(static_cast<int>(getUint("kSmartCurrentStallLimit")),
                                      static_cast<int>(getUint("kSmartCurrentFreeLimit")),
                                      static_cast<int>(getUint("kSmartCurrentConfig")));
    }

    // Closed-loop control.  Slot 0 is used for position (and current), slot 1
    // is used for velocity.  The built-in encoder is always used.
    sparkConfig.closedLoop.SetFeedbackSensor(FeedbackSensor::kPrimaryEncoder);

    auto slotConfig = [&](const std::string &suffix, const ClosedLoopSlot slot) -> void
    {
        if (config_.count("kP" + suffix) != 0)
        {
            sparkConfig.closedLoop.P(getDouble("kP" + suffix), slot);
        }
        if (config_.count("kI" + suffix) != 0)
        {
            sparkConfig.closedLoop.I(getDouble("kI" + suffix), slot);
        }
        if (config_.count("kD" + suffix) != 0)
        {
            sparkConfig.closedLoop.D(getDouble("kD" + suffix), slot);
        }
        if (config_.count("kF" + suffix) != 0)
        {
            // The 2024 velocity feedforward was in units of (duty cycle) output
            // per unit of velocity; kV is in Volts per unit of velocity.
            sparkConfig.closedLoop.feedForward.kV(getDouble("kF" + suffix) * 12.0, slot);
        }
        if (config_.count("kIZone" + suffix) != 0)
        {
            sparkConfig.closedLoop.IZone(getDouble("kIZone" + suffix), slot);
        }
        if (config_.count("kIMaxAccum" + suffix) != 0)
        {
            sparkConfig.closedLoop.IMaxAccum(getDouble("kIMaxAccum" + suffix), slot);
        }
        if (config_.count("kDFilter" + suffix) != 0)
        {
            sparkConfig.closedLoop.DFilter(getDouble("kDFilter" + suffix), slot);
        }
        if (config_.count("kOutputMin" + suffix) != 0 || config_.count("kOutputMax" + suffix) != 0)
        {
            sparkConfig.closedLoop.OutputRange(getDouble("kOutputMin" + suffix), getDouble("kOutputMax" + suffix), slot);
        }
        if (config_.count("kSmartMotionMaxVelocity" + suffix) != 0)
        {
            sparkConfig.closedLoop.maxMotion.CruiseVelocity(getDouble("kSmartMotionMaxVelocity" + suffix), slot);
        }
        if (config_.count("kSmartMotionMaxAccel" + suffix) != 0)
        {
            sparkConfig.closedLoop.maxMotion.MaxAcceleration(getDouble("kSmartMotionMaxAccel" + suffix), slot);
        }
        if (config_.count("kSmartMotionAllowedClosedLoopError" + suffix) != 0)
        {
            sparkConfig.closedLoop.maxMotion.AllowedProfileError(getDouble("kSmartMotionAllowedClosedLoopError" + suffix), slot);
        }
    };

    slotConfig("_0", ClosedLoopSlot::kSlot0);
    slotConfig("_1", ClosedLoopSlot::kSlot1);

    return reporting;
}

void SparkMotor::Configure(const bool burn) noexcept
{
    configGood_ = false;

    if (!motor_)
    {
        return;
    }

    SparkBaseConfig *sparkConfig{nullptr};
    SparkMaxConfig maxConfig;
    SparkFlexConfig flexConfig;

    if (flex_not_max_)
    {
        sparkConfig = &flexConfig;
    }
    else
    {
        sparkConfig = &maxConfig;
    }

    configReporting_ = BuildConfig(*sparkConfig);

    if (!configReporting_.empty())
    {
        std::printf("SparkMax[%i] %s config: %s.\n", canId_, name_.c_str(), configReporting_.c_str());
    }

    // Always reset all parameters to their factory defaults first, so that
    // only the managed parameters differ from the defaults.  This clears out
    // anything left on the controller by other code (inversion, follower
    // mode, current limits, etc.), which could otherwise make motors run
    // unexpectedly.  The first time the config is applied (at startup, when
    // the robot is disabled), it is also saved to flash, so the controller
    // comes back up correctly configured after a brownout or power cycle;
    // this is what REV recommends.  Otherwise, only save when asked.
    const bool persist = burn || !persisted_;
    const rev::ResetMode resetMode = rev::ResetMode::kResetSafeParameters;
    const rev::PersistMode persistMode = persist ? rev::PersistMode::kPersistParameters : rev::PersistMode::kNoPersistParameters;

    DoSafely("Configure", [&]() -> void
             {
        if (AnyError(motor_->Configure(*sparkConfig, resetMode, persistMode)))
        {
            std::printf("SparkMax[%i] %s %s config: FAIL (%i)!\n", canId_, name_.c_str(), persist ? "burn" : "apply", lastError_);

            // Saving to flash is refused while the robot is enabled; make
            // sure the config is at least applied.
            if (!persist || AnyError(motor_->Configure(*sparkConfig, resetMode, rev::PersistMode::kNoPersistParameters)))
            {
                return;
            }
        }
        else if (persist)
        {
            persisted_ = true;

            std::printf("SparkMax[%i] %s burn config: OK.\n", canId_, name_.c_str());
        }

        configGood_ = configReporting_.empty(); });
}

void SparkMotor::ClearFaults() noexcept
{
    if (faultBits_ != 0 ||
        warningBits_ != 0 ||
        resets_ != 0 ||
        catches_ != 0 ||
        errors_ != 0)
    {
        const bool status = GetStatus();

        std::printf("SparkMax[%i] %s Status: %s[%s|%s] resets=%u catches=%u errors=%u (last=%i).\n",
                    canId_,
                    name_.c_str(),
                    status ? "T" : "F",
                    FaultInfo(SparkBase::Faults(faultBits_)).c_str(),
                    WarningInfo(SparkBase::Warnings(warningBits_)).c_str(),
                    resets_,
                    catches_,
                    errors_,
                    lastError_);
    }

    faultBits_ = 0;
    warningBits_ = 0;
    resets_ = 0;
    catches_ = 0;
    errors_ = 0;
    lastError_ = 0;

    DoSafely("ClearFaults", [&]() -> void
             { if (motor_) { (void)AnyError(motor_->ClearFaults()); } });
}

bool SparkMotor::GetStatus() noexcept
{
    return motor_ && configGood_ && faultBits_ == 0;
}

void SparkMotor::ShuffleboardCreate(frc::ShuffleboardContainer &container,
                                  std::function<void(double)> control,
                                  std::function<void()> reset) noexcept
{
    temperatureUI_ = &container.Add("Temperature", 0.0)
                          .WithPosition(0, 0);

    statusUI_ = &container.Add("Status", false)
                     .WithPosition(0, 1)
                     .WithWidget(frc::BuiltInWidgets::kBooleanBox);

    faultsUI_ = &container.Add("Faults", "")
                     .WithPosition(1, 0);

    stickyFaultsUI_ = &container.Add("StickyFaults", "")
                           .WithPosition(1, 1);

    voltageUI_ = &container.Add("Voltage", 0.0)
                      .WithPosition(2, 0);

    currentUI_ = &container.Add("Current", 0.0)
                      .WithPosition(2, 1);

    percentUI_ = &container.Add("Percent (Actual)", 0.0)
                      .WithPosition(3, 0)
                      .WithWidget(frc::BuiltInWidgets::kNumberBar);

    speedUI_ = &container.Add("Speed (Commanded)", 0.0)
                    .WithPosition(3, 1)
                    .WithWidget(frc::BuiltInWidgets::kNumberBar);

    distanceUI_ = &container.Add("Distance (Rotations)", 0.0)
                       .WithPosition(4, 0);

    velocityUI_ = &container.Add("Velocity (Rot per sec)", 0.0)
                       .WithPosition(4, 1);

    controlUI_ = &container.Add("Control", 0.0)
                      .WithPosition(5, 0)
                      .WithWidget(frc::BuiltInWidgets::kNumberSlider);

    resetUI_ = &container.Add("Reset", false)
                    .WithPosition(5, 1)
                    .WithWidget(frc::BuiltInWidgets::kToggleButton);

    controlF_ = control;
    resetF_ = reset;
    shuffleboard_ = true;
}

void SparkMotor::ShuffleboardPeriodic() noexcept
{
    // Read controls information from Shuffleboard and manage interactive UI.
    const bool reset = resetUI_->GetEntry()->GetBoolean(false);
    double control = controlUI_->GetEntry()->GetDouble(0.0);

    if (reset)
    {
        resetUI_->GetEntry()->SetBoolean(false);
        controlUI_->GetEntry()->SetDouble(0.0);
        control = 0.0;
    }

    double temperature{0.0};
    std::string faults{"--------|--------"};
    std::string stickyFaults{"--------|--------"};
    bool anyFault{true};
    double voltage{0.0};
    double current{0.0};
    double percent{0.0};
    double speed{0.0};
    double distance{0.0};
    double velocity{0.0};

    // Obtain raw data from the SPARK.
    DoSafely("ShuffleboardPeriodic", [&]() -> void
             {
        if (motor_)
        {
            temperature = motor_->GetMotorTemperature();
            faults = FaultInfo(motor_->GetFaults()) + "|" + WarningInfo(motor_->GetWarnings());
            stickyFaults = FaultInfo(motor_->GetStickyFaults()) + "|" + WarningInfo(motor_->GetStickyWarnings());
            anyFault = motor_->HasActiveFault();
            voltage = motor_->GetBusVoltage();
            current = motor_->GetOutputCurrent();
            percent = motor_->GetAppliedOutput(); // Actual setting
            speed = motor_->Get();                // Commanded setting
            distance = motor_->GetEncoder().GetPosition();        // Rotations (unless converted)
            velocity = motor_->GetEncoder().GetVelocity() / 60.0; // Rotations per second (unless converted)
        } });

    temperatureUI_->GetEntry()->SetDouble(temperature);
    statusUI_->GetEntry()->SetBoolean(motor_ && configGood_ && !anyFault);
    faultsUI_->GetEntry()->SetString(faults);
    stickyFaultsUI_->GetEntry()->SetString(stickyFaults);
    voltageUI_->GetEntry()->SetDouble(voltage);
    currentUI_->GetEntry()->SetDouble(current);
    percentUI_->GetEntry()->SetDouble(percent);
    speedUI_->GetEntry()->SetDouble(speed);
    distanceUI_->GetEntry()->SetDouble(distance);
    velocityUI_->GetEntry()->SetDouble(velocity);

    if (reset)
    {
        Stop();

        if (resetF_)
        {
            resetF_();
        }
        else
        {
            // Logic above ensures that `position` is now zero; reset the
            // motor encoder to reflect this.
            SpecifyPosition(0.0);
            ClearFaults();
        }
    }

    if (control != previousControl_)
    {
        previousControl_ = control;

        if (controlF_)
        {
            controlF_(control);
        }
        else
        {
            Set(control);
        }
    }
}

// Periodic() normally runs at 50Hz, but this is a default and it is possible
// that this is running at another rate.  So, GetFPGATime() is used to work
// things out (more below), for management of any controller reset.
void SparkMotor::Periodic() noexcept
{
    // Do any Shuffleboard work, if Shuffleboard has been set up.
    if (shuffleboard_)
    {
        ShuffleboardPeriodic();
    }

    if (!motor_)
    {
        return;
    }

    // These calls are safe and "free", in that they only report information
    // collected from periodic status frames.  To capture any transient
    // events, sticky faults/warnings are used.  There is no expectation that
    // this information is instantaneously accurate, it is only used for
    // reporting and to detect any motor controller reboots.  This is cleared
    // by ClearFaults().
    bool hasReset{false};

    DoSafely("GetStickyFaults/GetStickyWarnings", [&]() -> void
             {
        const SparkBase::Faults faults = motor_->GetStickyFaults();
        const SparkBase::Warnings warnings = motor_->GetStickyWarnings();

        faultBits_ |= faults.rawBits;
        warningBits_ |= warnings.rawBits;
        hasReset = warnings.hasReset; });

    // In microseconds.  At 50Hz, this will nominally tick by increments of
    // 20000.  The object here is to only continue at a rate of around 3Hz, and
    // to spread things around so that the aggregate work of all motor
    // controllers is done evenly; to avoid synchronizing many motor
    // controllers so they all do this work during the same period.  The CAN ID
    // (mod 16) is used to facilitate this spread, along with a counter to
    // track Periodic() calls.
    const uint64_t FPGATime = frc::RobotController::GetFPGATime();
    const uint64_t deltaTime = FPGATime - FPGATime_;

    if (++iteration_ % 16 != canId_ % 16 || deltaTime < 250000)
    {
        return;
    }

    FPGATime_ = FPGATime;
    iteration_ = canId_ % 16;

    // If the controller has reset (power loss, brownout, etc.), clear the
    // sticky faults/warnings so this is detected only once, and re-apply the
    // config, in case it was not persisted.
    if (hasReset)
    {
        ++resets_;

        std::printf("SparkMax[%i] %s reset detected; re-applying config.\n", canId_, name_.c_str());

        DoSafely("ClearFaults", [&]() -> void
                 { (void)AnyError(motor_->ClearFaults()); });

        Configure(false);
    }
}

void SparkMotor::SetIdleMode(const SmartMotorBase::IdleMode mode) noexcept
{
    if (mode == idleMode_)
    {
        return;
    }

    idleMode_ = mode;

    if (!motor_)
    {
        return;
    }

    // Only change the idle mode (leave all other parameters alone).
    SparkBaseConfig *sparkConfig{nullptr};
    SparkMaxConfig maxConfig;
    SparkFlexConfig flexConfig;

    if (flex_not_max_)
    {
        sparkConfig = &flexConfig;
    }
    else
    {
        sparkConfig = &maxConfig;
    }

    sparkConfig->SetIdleMode(mode == IdleMode::kBrake ? SparkBaseConfig::IdleMode::kBrake : SparkBaseConfig::IdleMode::kCoast);

    DoSafely("SetIdleMode", [&]() -> void
             { (void)AnyError(motor_->Configure(*sparkConfig, rev::ResetMode::kNoResetSafeParameters, rev::PersistMode::kNoPersistParameters)); });
}

SmartMotorBase::IdleMode SparkMotor::GetIdleMode() noexcept
{
    return idleMode_;
}

void SparkMotor::EnableLimit(const Direction direction) noexcept
{
    switch (direction)
    {
    case (Direction::kForward):
        config_["kHardLimitFwdEn"] = bool{true};
        break;
    case (Direction::kReverse):
        config_["kHardLimitRevEn"] = bool{true};
        break;
    }

    Configure(false);
}

void SparkMotor::DisableLimit(const Direction direction) noexcept
{
    switch (direction)
    {
    case (Direction::kForward):
        config_["kHardLimitFwdEn"] = bool{false};
        break;
    case (Direction::kReverse):
        config_["kHardLimitRevEn"] = bool{false};
        break;
    }

    Configure(false);
}

bool SparkMotor::GetLimit(const Direction direction) noexcept
{
    bool result = false;

    DoSafely("GetLimit", [&]() -> void
             {
        if (!motor_)
        {
            return;
        }

        switch (direction)
        {
        case (Direction::kForward):
            result = motor_->GetForwardLimitSwitch().Get();
            break;
        case (Direction::kReverse):
            result = motor_->GetReverseLimitSwitch().Get();
            break;
        } });

    return result;
}

void SparkMotor::Stop() noexcept
{
    position_ = 0.0;
    velocity_ = 0.0;

    DoSafely("Stop", [&]() -> void
             { if (motor_) { motor_->StopMotor(); } });
}

void SparkMotor::Set(const double percent) noexcept
{
    position_ = 0.0;
    velocity_ = 0.0;

    DoSafely("Set", [&]() -> void
             { if (motor_) { motor_->Set(percent); } });
}

double SparkMotor::Get() noexcept
{
    double result = 0.0;

    DoSafely("Get", [&]() -> void
             { if (motor_) { result = motor_->Get(); } });

    return result;
}

// This is based on voltage compensation, rather than feedback.
void SparkMotor::SetVoltage(const units::volt_t voltage) noexcept
{
    position_ = 0.0;
    velocity_ = 0.0;

    DoSafely("SetVoltage", [&]() -> void
             { if (motor_) { motor_->SetVoltage(voltage); } });
}

// Note that current control uses PID parameter set 0, which is normally used
// for position.  It is unlikely both current and position control would be
// used with the same mechanism, so this is OK.  Any error is tracked, but is
// not propagated from this context (similar to Set).
void SparkMotor::SetCurrent(const units::ampere_t current) noexcept
{
    position_ = 0.0;
    velocity_ = 0.0;

    DoSafely("SetCurrent", [&]() -> void
             { if (motor_) { (void)AnyError(motor_->GetClosedLoopController().SetSetpoint(current.value(), SparkBase::ControlType::kCurrent, ClosedLoopSlot::kSlot0)); } });
}

units::volt_t SparkMotor::GetVoltage() noexcept
{
    double voltage = 0.0;

    DoSafely("GetBusVoltage/GetAppliedOutput", [&]() -> void
             { if (motor_) { voltage = motor_->GetBusVoltage() * motor_->GetAppliedOutput(); } });

    return units::volt_t{voltage};
}

units::ampere_t SparkMotor::GetCurrent() noexcept
{
    double current = 0.0;

    DoSafely("GetOutputCurrent", [&]() -> void
             { if (motor_) { current = motor_->GetOutputCurrent(); } });

    return units::ampere_t{current};
}

// Note that any error is tracked, but is not propagated from this context.
void SparkMotor::SpecifyPosition(const double position) noexcept
{
    DoSafely("SpecifyPosition", [&]() -> void
             { if (motor_) { (void)AnyError(motor_->GetEncoder().SetPosition(position)); } });
}

// SmartMotion (2024) has been replaced by MAXMotion.
void SparkMotor::SeekPosition(const double position) noexcept
{
    position_ = position;
    velocity_ = 0.0;

    DoSafely("SeekPosition", [&]() -> void
             { if (motor_) { (void)AnyError(motor_->GetClosedLoopController().SetSetpoint(position, SparkBase::ControlType::kMAXMotionPositionControl, ClosedLoopSlot::kSlot0)); } });
}

bool SparkMotor::CheckPosition(const double tolerance) noexcept
{
    double error = GetPositionRaw() - position_;

    return error >= -tolerance && error < tolerance;
}

double SparkMotor::GetPositionRaw() noexcept
{
    double result = 0.0;

    DoSafely("GetPosition", [&]() -> void
             { if (motor_) { result = motor_->GetEncoder().GetPosition(); } });

    return result;
}

void SparkMotor::SeekVelocity(const double velocity) noexcept
{
    position_ = 0.0;
    velocity_ = velocity;

    DoSafely("SeekVelocity", [&]() -> void
             { if (motor_) { (void)AnyError(motor_->GetClosedLoopController().SetSetpoint(velocity, SparkBase::ControlType::kMAXMotionVelocityControl, ClosedLoopSlot::kSlot1)); } });
}

bool SparkMotor::CheckVelocity(const double tolerance) noexcept
{
    double error = GetVelocityRaw() - velocity_;

    return error >= -tolerance && error < tolerance;
}

double SparkMotor::GetVelocityRaw() noexcept
{
    double result = 0.0;

    DoSafely("GetVelocity", [&]() -> void
             { if (motor_) { result = motor_->GetEncoder().GetVelocity(); } });

    return result;
}

bool SparkMotor::AnyError(const rev::REVLibError returnCode) noexcept
{
    if (returnCode == rev::REVLibError::kOk)
    {
        return false;
    }

    ++errors_;
    lastError_ = static_cast<int>(returnCode);

    return true;
}

// REVLib is closed source (and does not specify "noexcept"), so it is
// untrusted, in that it is surrounded by try/catch.  In the event of an
// exception, the motor controller is abandoned (but the rest of the robot
// code continues to run normally).
void SparkMotor::DoSafely(const char *const what, std::function<void()> work) noexcept
{
    try
    {
        work();
    }
    catch (const std::exception &e)
    {
        motor_ = nullptr;
        max_motor_ = nullptr;
        flex_motor_ = nullptr;

        ++catches_;

        std::printf("SparkMax[%i] %s %s exception: %s.\n", canId_, name_.c_str(), what, e.what());
    }
    catch (...)
    {
        motor_ = nullptr;
        max_motor_ = nullptr;
        flex_motor_ = nullptr;

        ++catches_;

        std::printf("SparkMax[%i] %s %s unknown exception.\n", canId_, name_.c_str(), what);
    }
}
