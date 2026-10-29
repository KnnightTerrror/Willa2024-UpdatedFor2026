#pragma once

#include <string_view>

#include "infrastructure/SmartMotor.h"

namespace SparkMaxFactory
{
    // Print out the list of managed config parameters (with default values).
    void ConfigIndex() noexcept;

    // Create a SPARK MAX/FLEX motor controller.
    std::unique_ptr<SmartMotorBase> CreateSparkMax(const std::string_view name, const int canId, const bool inverted) noexcept;
    std::unique_ptr<SmartMotorBase> CreateSparkFlex(const std::string_view name, const int canId, const bool inverted) noexcept;
}

// Configuration (REVLib 2026):

// Starting with REVLib 2025, REV replaced the old "get/set one parameter at a
// time" API (SetP(), SetSmartCurrentLimit(), SetPeriodicFramePeriod(),
// RestoreFactoryDefaults(), BurnFlash(), etc.) with configuration objects
// (rev::spark::SparkMaxConfig / SparkFlexConfig) that are applied all at once
// via Configure().  The 2024 code here used a fairly involved state machine
// to read back, compare, set, and burn individual parameters over many
// Periodic() iterations.  That is no longer possible (or needed): this code
// now keeps the same string-keyed ConfigMap interface (so subsystem code does
// not need to change), but translates the map into a REVLib config object and
// applies it with a single Configure() call.

// Every ApplyConfig() first resets all "safe" parameters to factory defaults
// and then applies the config, so the controller always ends up with exactly
// this config (and nothing left over from other code).  The first
// ApplyConfig() (at construction, while the robot is disabled) also saves the
// config to flash, as REV recommends, so the controller comes up correctly
// configured after a power cycle.  After that, ApplyConfig(false) does not
// save (it is used whenever settings are adjusted from Test Mode), while
// ApplyConfig(true) does (this replaces the old RestoreFactoryDefaults() +
// BurnFlash() sequence).  REVLib refuses to save parameters while the robot
// is enabled, so only burn when disabled (e.g. on leaving Test Mode).

// Since every parameter not listed via SetConfig()/AddConfig() is reset to
// the factory default, there is no reason to list parameters that should be
// at their default values.

// The motor controller may reset (brownout, etc.).  Periodic() watches for
// the "has reset" warning; when it is seen, faults are cleared and the config
// is re-applied, so anything that was not persisted is restored.

// The table below documents the managed configuration parameters, with their
// types and default values (this is also the list of keys accepted by
// SetConfig()/AddConfig(); any other key is reported as invalid).  Each is
// mapped to the REVLib 2026 setting noted in the comment.
namespace SparkMaxFactory
{
    // **** DO NOT ALTER THIS LIST WITHOUT UNDERSTANDING THE IMPLICATIONS! ****
    // In particular, std::map is sorted, so the order here is meaningless.
    const SmartMotorBase::ConfigMap configDefaults = {
        {"kStatus0", uint{10}},                               // ms; signals: applied output, faults, warnings
        {"kStatus1", uint{20}},                               // ms; signals: velocity, bus voltage, current, temperature
        {"kStatus2", uint{20}},                               // ms; signals: position
        {"kIdleMode", uint{1}},                               // 0 = Coast, 1 = Brake
        {"kLimitSwitchFwdPolarity", uint{0}},                 // 0 = Normally Open, 1 = Normally Closed
        {"kLimitSwitchRevPolarity", uint{0}},                 // 0 = Normally Open, 1 = Normally Closed
        {"kHardLimitFwdEn", bool{true}},                      // limit switch stops motor
        {"kHardLimitRevEn", bool{true}},                      // limit switch stops motor
        {"kFollowerID", uint{0}},                             // 0x02051800 | <CAN ID of leader>, or 0
        {"kFollowerConfig", uint{0}},                         // | 0x00004000 to invert, when following
        {"kSoftLimitFwd", double{0.0}},                       // softLimit.ForwardSoftLimit()
        {"kSoftLimitRev", double{0.0}},                       // softLimit.ReverseSoftLimit()
        {"kRampRate", double{0.0}},                           // OpenLoopRampRate()
        {"kClosedLoopRampRate", double{0.0}},                 // ClosedLoopRampRate()
        {"kCompensatedNominalVoltage", double{0.0}},          // VoltageCompensation(); 0 = disabled
        {"kEncoderAverageDepth", uint{8}},                    // encoder.UvwAverageDepth() [1, 8]
        {"kEncoderSampleDelta", uint{32}},                    // encoder.UvwMeasurementPeriod() ms
        {"kPositionConversionFactor", double{1.0}},           // encoder.PositionConversionFactor()
        {"kVelocityConversionFactor", double{1.0}},           // encoder.VelocityConversionFactor()
        {"kCurrentChop", double{0.0}},                        // SecondaryCurrentLimit(); 0 = disabled
        {"kCurrentChopCycles", uint{0}},                      // SecondaryCurrentLimit()
        {"kSmartCurrentStallLimit", uint{80}},                // Amps; SmartCurrentLimit()
        {"kSmartCurrentFreeLimit", uint{20}},                 // Amps; SmartCurrentLimit()
        {"kSmartCurrentConfig", uint{10000}},                 // RPM; SmartCurrentLimit()
        {"kP_0", double{0.0}},                                // closedLoop, slot 0 (position)
        {"kI_0", double{0.0}},
        {"kD_0", double{0.0}},
        {"kF_0", double{0.0}},                                // closedLoop.feedForward.kV() = kF * 12V
        {"kIZone_0", double{0.0}},
        {"kIMaxAccum_0", double{0.0}},
        {"kDFilter_0", double{0.0}},
        {"kOutputMin_0", double{-1.0}},
        {"kOutputMax_0", double{1.0}},
        {"kSmartMotionMaxVelocity_0", double{0.0}},           // maxMotion.CruiseVelocity()
        {"kSmartMotionMaxAccel_0", double{0.0}},              // maxMotion.MaxAcceleration()
        {"kSmartMotionMinVelOutput_0", double{0.0}},          // No MAXMotion equivalent; ignored
        {"kSmartMotionAllowedClosedLoopError_0", double{0.0}}, // maxMotion.AllowedProfileError()
        {"kSmartMotionAccelStrategy_0", uint{0}},             // No MAXMotion equivalent; ignored
        {"kP_1", double{0.0}},                                // closedLoop, slot 1 (velocity)
        {"kI_1", double{0.0}},
        {"kD_1", double{0.0}},
        {"kF_1", double{0.0}},
        {"kIZone_1", double{0.0}},
        {"kIMaxAccum_1", double{0.0}},
        {"kDFilter_1", double{0.0}},
        {"kOutputMin_1", double{-1.0}},
        {"kOutputMax_1", double{1.0}},
        {"kSmartMotionMaxVelocity_1", double{0.0}},
        {"kSmartMotionMaxAccel_1", double{0.0}},
        {"kSmartMotionMinVelOutput_1", double{0.0}},
        {"kSmartMotionAllowedClosedLoopError_1", double{0.0}},
        {"kSmartMotionAccelStrategy_1", uint{0}},
    };
}
