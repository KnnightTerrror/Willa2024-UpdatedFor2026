#include "subsystems/IntakeSubsystem.h"

#include <frc/DriverStation.h>
#include <frc/RobotController.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <units/voltage.h>

#include <cmath>
#include <cstdio>

#include <array>
#include <string>

IntakeSubsystem::IntakeSubsystem() noexcept
{
    // NEO 550.
    const SmartMotorBase::ConfigMap config = {
        {"kStatus1", uint{250}},
        {"kStatus2", uint{250}},
        {"kIdleMode", uint{0}},
        {"kRampRate", double{0.1}},
        {"kSmartCurrentStallLimit", uint{25}}, // Amps
        {"kSmartCurrentFreeLimit", uint{25}},  // Amps
    };

    // neos
    const SmartMotorBase::ConfigMap neoConfig = {
        {"kStatus1", uint{250}},
        {"kStatus2", uint{250}},
        {"kIdleMode", uint{1}},
        {"kRampRate", double{0.5}},
        {"kSmartCurrentStallLimit", uint{40}}, // Amps
        // {"kSoftLimitFwd", double{0.0}},
        // {"kSoftLimitRev", double{0.0}},
    };

    // The 2024 code set a 1 A stall current limit for the intake, which is far
    // too low for the intake to start turning (a motor starting from rest draws
    // stall current).  Use the same NEO 550 limit as the trigger.
    const SmartMotorBase::ConfigMap intakeConfig = {
        {"kSmartCurrentStallLimit", uint{25}}, // Amps
    };

    const SmartMotorBase::ConfigMap flexConfig = {
        {"kStatus1", uint{250}},
        {"kStatus2", uint{250}},
        {"kRampRate", double{0.5}},
        {"kSmartCurrentStallLimit", uint{nonDrive::kShooterStallCurrentLimit}},
        {"kSmartCurrentFreeLimit", uint{nonDrive::kShooterFreeCurrentLimit}},
        {"kCurrentChop", double{nonDrive::kShooterSecondaryCurrentLimit}},
        // {"kSoftLimitFwd", double{0.0}},
        // {"kSoftLimitRev", double{0.0}},
    };

    const SmartMotorBase::ConfigMap climberConfig = {
        {"kIdleMode", uint32_t{1}}, // Brake.
    };

    // shoulderMotorBase_ = SparkMaxFactory::CreateSparkMax("Shoulder", nonDrive::kShoulderMotorCanID, nonDrive::kShoulderMotorInverted);
    m_intakeMotor = SparkMaxFactory::CreateSparkMax("Intake", nonDrive::kIntakeMotorCanID, nonDrive::kIntakeMotorInverted);
    m_triggerMotor = SparkMaxFactory::CreateSparkMax("Trigger", nonDrive::kTriggerMotorCanID, nonDrive::kTriggerMotorInverted);
    m_shooter1Motor = SparkMaxFactory::CreateSparkFlex("Shooter1", nonDrive::kShooter1MotorCanID, nonDrive::kShooter1MotorInverted); // invert all of thes
    m_shooter2Motor = SparkMaxFactory::CreateSparkFlex("Shooter2", nonDrive::kShooter2MotorCanID, nonDrive::kShooter2MotorInverted);
    m_climber1Motor = SparkMaxFactory::CreateSparkMax("Climber1", nonDrive::kClimber1MotorCanID, nonDrive::kClimber1MotorInverted);
    m_climber2Motor = SparkMaxFactory::CreateSparkMax("Climber2", nonDrive::kClimber2MotorCanID, nonDrive::kClimber2MotorInverted);

    m_intakeMotor->SetConfig(config);
    m_intakeMotor->AddConfig(intakeConfig);
    m_triggerMotor->SetConfig(config);
    m_shooter1Motor->SetConfig(flexConfig);
    m_shooter2Motor->SetConfig(flexConfig);
    m_climber1Motor->SetConfig(neoConfig);
    m_climber1Motor->AddConfig(climberConfig);
    m_climber2Motor->SetConfig(neoConfig);
    m_climber2Motor->AddConfig(climberConfig);

    m_intakeMotor->ApplyConfig(false);
    m_triggerMotor->ApplyConfig(false);
    m_shooter1Motor->ApplyConfig(false);
    m_shooter2Motor->ApplyConfig(false);
    m_climber1Motor->ApplyConfig(false);
    m_climber2Motor->ApplyConfig(false);

    Default(0.0);
}

void IntakeSubsystem::Periodic() noexcept
{
    m_intakeMotor->Periodic();
    m_triggerMotor->Periodic();
    m_shooter1Motor->Periodic();
    m_shooter2Motor->Periodic();
    m_climber1Motor->Periodic();
    m_climber2Motor->Periodic();

    MonitorShooter();
    PublishTelemetry();
}

// Shooter protection.  The current limits are set on the motor controllers
// (see the constructor), but these only take effect if the controller actually
// received its config, and a damaged motor or controller can still draw far
// too much.  So, in code: (1) a shooter motor whose config was not confirmed is
// not run, and (2) a shooter motor drawing more than kShooterTripCurrent for
// longer than kShooterTripTime is stopped, until the robot is disabled.
void IntakeSubsystem::MonitorShooter() noexcept
{
    const std::array<SmartMotorBase *, 2> motors{m_shooter1Motor.get(), m_shooter2Motor.get()};
    const std::array<const char *, 2> names{"Shooter1", "Shooter2"};

    // Disabling the robot clears any trip, so the next enable tries again.
    if (frc::DriverStation::IsDisabled())
    {
        for (std::size_t i = 0; i < motors.size(); ++i)
        {
            if (m_shooterTripped[i])
            {
                std::printf("%s current trip cleared (robot disabled).\n", names[i]);
            }

            m_shooterTripped[i] = false;
            m_shooterOverCurrentSince[i] = 0;
        }

        return;
    }

    const uint64_t FPGATime = frc::RobotController::GetFPGATime();
    const uint64_t tripTime = static_cast<uint64_t>(nonDrive::kShooterTripTime.value() * 1000000.0);

    for (std::size_t i = 0; i < motors.size(); ++i)
    {
        const double current = std::abs(motors[i]->GetCurrent().value());

        if (current < nonDrive::kShooterTripCurrent)
        {
            m_shooterOverCurrentSince[i] = 0;
        }
        else if (m_shooterOverCurrentSince[i] == 0)
        {
            m_shooterOverCurrentSince[i] = FPGATime;
        }
        else if (!m_shooterTripped[i] && FPGATime - m_shooterOverCurrentSince[i] > tripTime)
        {
            m_shooterTripped[i] = true;
            motors[i]->Stop();

            std::printf("**** %s STOPPED: drawing %.0f A (over %.0f A for %.2f s).  It stays off until the robot "
                        "is disabled.  Check the motor, its SPARK Flex, and the shooter mechanism.\n",
                        names[i], current, nonDrive::kShooterTripCurrent, nonDrive::kShooterTripTime.value());
        }
    }
}

// Why a shooter motor is not being run (empty if it can run).
std::string IntakeSubsystem::ShooterBlockedReason(const std::size_t index) noexcept
{
    SmartMotorBase *const motor = index == 0 ? m_shooter1Motor.get() : m_shooter2Motor.get();

    if (m_shooterTripped[index])
    {
        return "Current trip (until disabled)";
    }

    if (!motor->IsConfigured())
    {
        return "Config not confirmed (current limits not in effect)";
    }

    return "";
}

// All shooter output goes through here, so the protection above is enforced.
void IntakeSubsystem::SetShooterVoltage(const units::volt_t voltage) noexcept
{
    const std::array<SmartMotorBase *, 2> motors{m_shooter1Motor.get(), m_shooter2Motor.get()};
    const std::array<const char *, 2> names{"Shooter1", "Shooter2"};

    for (std::size_t i = 0; i < motors.size(); ++i)
    {
        const std::string reason = ShooterBlockedReason(i);

        if (reason.empty())
        {
            motors[i]->SetVoltage(voltage);
        }
        else
        {
            motors[i]->Stop();

            std::printf("%s not run: %s.\n", names[i], reason.c_str());
        }
    }
}

// Motor diagnostics, under Intake/ on the dashboard (SmartDashboard), 10 times
// a second.  For each motor: Volts (applied to the motor), Current (A), Speed
// (RPM), Bus Volts (battery voltage at the motor controller), Temp (C), and
// Status (motor controller present, configured and without faults).
void IntakeSubsystem::PublishTelemetry() noexcept
{
    if (++m_telemetryCounter < 5)
    {
        return;
    }

    m_telemetryCounter = 0;

    struct Keys
    {
        std::string volts, current, speed, busVolts, temperature, status;
    };

    static const std::array<std::string, 6> names{"Intake", "Trigger", "Shooter1", "Shooter2", "Climber1", "Climber2"};

    static const std::array<Keys, 6> keys = []() -> std::array<Keys, 6>
    {
        std::array<Keys, 6> result;

        for (std::size_t i = 0; i < names.size(); ++i)
        {
            const std::string prefix = "Intake/" + names[i] + "/";

            result[i] = Keys{prefix + "Volts", prefix + "Current (A)", prefix + "Speed (RPM)",
                             prefix + "Bus Volts", prefix + "Temp (C)", prefix + "Status"};
        }

        return result;
    }();

    const std::array<SmartMotorBase *, 6> motors{
        m_intakeMotor.get(),
        m_triggerMotor.get(),
        m_shooter1Motor.get(),
        m_shooter2Motor.get(),
        m_climber1Motor.get(),
        m_climber2Motor.get()};

    for (std::size_t i = 0; i < motors.size(); ++i)
    {
        frc::SmartDashboard::PutNumber(keys[i].volts, motors[i]->GetVoltage().value());
        frc::SmartDashboard::PutNumber(keys[i].current, motors[i]->GetCurrent().value());
        frc::SmartDashboard::PutNumber(keys[i].speed, motors[i]->GetVelocityRaw());
        frc::SmartDashboard::PutNumber(keys[i].busVolts, motors[i]->GetBusVoltage().value());
        frc::SmartDashboard::PutNumber(keys[i].temperature, motors[i]->GetTemperature());
        frc::SmartDashboard::PutBoolean(keys[i].status, motors[i]->GetStatus());
    }

    // Shooter protection (see MonitorShooter()); empty means OK.
    frc::SmartDashboard::PutString("Intake/Shooter1/Blocked", ShooterBlockedReason(0));
    frc::SmartDashboard::PutString("Intake/Shooter2/Blocked", ShooterBlockedReason(1));
}

void IntakeSubsystem::TestInit() noexcept {}
void IntakeSubsystem::TestExit() noexcept {}
void IntakeSubsystem::TestPeriodic() noexcept {}
void IntakeSubsystem::Default(const double percent) noexcept
{
    /*
     m_intakeMotor->SetVoltage(12.0_V);
     m_triggerMotor->SetVoltage(12.0_V);
     */
    m_intakeMotor->SetVoltage(4.0_V); // set power a lot lower
                                      // m_triggerMotor->Stop();

    //// m_shooter1Motor->Stop();
    // m_shooter2Motor->Stop();
    // m_climber1Motor->Stop();
    // m_climber2Motor->Stop();
}

// bottom
void IntakeSubsystem::RunIntake() noexcept
{
    m_intakeMotor->SetVoltage(12.0_V);

    m_triggerMotor->Stop();
    m_climber1Motor->Stop();
    m_climber2Motor->Stop();
    // Don't stop the shooter, so it can be spun up while intaking.
    // m_shooter1Motor->Stop();
    // m_shooter2Motor->Stop();
}

void IntakeSubsystem::StopIntake() noexcept
{
    m_intakeMotor->Stop();
    // m_triggerMotor->Stop();
    /*m_climber1Motor->Stop();
    m_climber2Motor->Stop();
    m_shooter1Motor->Stop();
    m_shooter2Motor->Stop();
    */
}

void IntakeSubsystem::ReverseIntake() noexcept
{
    m_intakeMotor->SetVoltage(-4.0_V);
    // m_triggerMotor->SetVoltage(12.0_V);
    // m_shooter1Motor->SetVoltage(-2.0_V);
    // m_shooter2Motor->SetVoltage(-2.0_V);

    m_climber1Motor->Stop();
    m_climber2Motor->Stop();
    m_shooter1Motor->Stop();
    m_shooter2Motor->Stop();
}
// Top

void IntakeSubsystem::FullOotas() noexcept
{
    SetShooterVoltage(12.0_V);
}

void IntakeSubsystem::Ootas() noexcept
{
    SetShooterVoltage(7.0_V);

    // if (counter <= 80)
    //{
    //    m_triggerMotor->SetVoltage(12.0_V);}
}

void IntakeSubsystem::ReverseOotas() noexcept
{
    m_triggerMotor->SetVoltage(-12.0_V);
    SetShooterVoltage(-6.0_V);
}

void IntakeSubsystem::StopOotas() noexcept
{
    m_shooter1Motor->Stop();
    m_shooter2Motor->Stop();
}

void IntakeSubsystem::StopOotasandBodyT() noexcept
{
    m_shooter1Motor->Stop();
    m_shooter2Motor->Stop();
    m_triggerMotor->Stop();
}

void IntakeSubsystem::RunTrigger() noexcept
{
    m_triggerMotor->SetVoltage(12.0_V);
}

void IntakeSubsystem::StopTrigger() noexcept
{
    m_triggerMotor->Stop();
}

void IntakeSubsystem::Climb1() noexcept
{
    m_climber1Motor->SetVoltage(6.0_V);

    m_intakeMotor->Stop();
}

void IntakeSubsystem::Climb2() noexcept
{

    m_climber2Motor->SetVoltage(6.0_V);

    m_intakeMotor->Stop();
}

void IntakeSubsystem::StopClimb1() noexcept
{
    m_climber1Motor->Stop();
}

void IntakeSubsystem::StopClimb2() noexcept
{
    m_climber2Motor->Stop();
}

void IntakeSubsystem::ReverseClimb1() noexcept
{
    m_climber1Motor->SetVoltage(-6.0_V);

    m_intakeMotor->Stop();
}

void IntakeSubsystem::ReverseClimb2() noexcept
{
    m_climber2Motor->SetVoltage(-6.0_V);

    m_intakeMotor->Stop();
}
