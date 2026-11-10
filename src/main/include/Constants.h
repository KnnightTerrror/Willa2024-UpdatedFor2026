#pragma once

#include <units/angle.h>
#include <units/angular_acceleration.h>
#include <units/angular_velocity.h>
#include <units/length.h>
#include <units/velocity.h>
#include <units/mass.h>
#include <units/torque.h>
#include <units/base.h>
#include <units/time.h>
#include <units/voltage.h>

#include <array>

namespace physical
{
    // Alignment constants, for each swerve module.  Specified on [-2048, 2048)
    // "count" scale, in (dimensionless) angular units.
    constexpr int kFrontLeftAlignmentOffset = +1427;
    constexpr int kFrontRightAlignmentOffset = -903;
    constexpr int kRearLeftAlignmentOffset = +384;
    constexpr int kRearRightAlignmentOffset = -205;

    // SDS Mk3 Standard (or Fast) Gear Ratio: 8.16:1 (or 6.86:1);
    // Nominal Wheel Diameter (4"): =0.1016m;
    // Nominal Wheel Circumference (pi * Diameter): ~0.3192m;
    // 8.16 / 0.3192 => ~25.57.

    // This should be empirically determined!  This is just an initial guess.
    // This is used for both distance and velocity control.  If this is off, it
    // will throw off kMaxDriveSpeed and kMaxTurnRate, as well as drive values.
    constexpr units::meter_t kDriveMetersPerRotation = 1.0_m / 21.15;

    // SDS Mk3 Standard (or Fast) Max Free Speed: 12.1 (or 14.4) feet/second;
    // This is an upper bound, for various reasons.  It needs to be empirically
    // measured.  Half of theoretical free speed is a reasonable starting value
    // (since something in the ballpark is needed here in order to to drive).
    constexpr units::meters_per_second_t kMaxDriveSpeed = 50.0_fps;

    // For a square drive base, with +/-11.25" x/y coordinates for each of four
    // swerve modules, the radius of the circle going through all modules is:
    // sqrt((11.25")^2 + (11.25")^2) ~= 15.91"; the circumference of such a
    // circle is 2*pi*15.91" ~= 99.96".

    // This is used for rotating the robot in place, about it's center.  This
    // may need to be empirically adjusted, but check kDriveMetersPerRotation
    // before making any adjustment here.
    constexpr units::meter_t kDriveMetersPerTurningCircle = 105.518_in;

    // This is the maximum rotational speed -- not of a swerve module, but of
    // the entire robot.  This is a function of the maximum drive speed and the
    // geometry of the robot.  This will occur when the robot spins in place,
    // around the center of a circle which passes through all the drive modules
    // (if there is no single such circle, things are analogous).  If the drive
    // modules are turned to be tangential to this circle and run at maximum,
    // the robot is rotating as fast as possible.  This can be derived from
    // kMaxDriveSpeed and the geometry and does not have to be directly
    // measured.  It is a good idea to check this value empirically though.

    // So the maximum rotational velocity (spinning in place) is kMaxDriveSpeed
    // / kDriveMetersPerTurningCircle * 360 degrees.  This should not need to
    // be empirically adjusted (but check).
    constexpr units::degrees_per_second_t kMaxTurnRate =
        kMaxDriveSpeed / kDriveMetersPerTurningCircle * 360.0_deg;

    // Drivebase geometry: distance between centers of right and left wheels on
    // robot; distance between centers of front and back wheels on robot.
    constexpr units::meter_t kTrackWidth = 23.75_in;
    constexpr units::meter_t kWheelBase = 23.75_in;

    // CAN ID and Digital I/O Port assignments.
    constexpr int kFrontLeftDriveMotorCanID = 1;
    constexpr int kFrontLeftTurningMotorCanID = 2;
    constexpr int kFrontRightDriveMotorCanID = 3;
    constexpr int kFrontRightTurningMotorCanID = 4;
    constexpr int kRearLeftDriveMotorCanID = 5;
    constexpr int kRearLeftTurningMotorCanID = 6;
    constexpr int kRearRightDriveMotorCanID = 7;
    constexpr int kRearRightTurningMotorCanID = 8;
    constexpr int kFrontLeftTurningEncoderPort = 0;
    constexpr int kFrontRightTurningEncoderPort = 1;
    constexpr int kRearLeftTurningEncoderPort = 2;
    constexpr int kRearRightTurningEncoderPort = 3;

    // These can flip because of gearing.
    constexpr bool kRightDriveMotorInverted = true;
    constexpr bool kLeftDriveMotorInverted = false;
    constexpr bool kTurningMotorInverted = true;
}

namespace pidf
{
    constexpr units::degrees_per_second_t kTurningPositionMaxVelocity = 1250.0_deg_per_s;
    constexpr units::degrees_per_second_squared_t kTurningPositionMaxAcceleration = 12500.0_deg_per_s_sq;
    constexpr double kTurningPositionP = 0.005;
    constexpr double kTurningPositionF = 0.003;
    constexpr double kTurningPositionI = 0.0;
    constexpr double kTurningPositionIZ = 0.0;
    constexpr double kTurningPositionIM = 0.0;
    constexpr double kTurningPositionD = 0.0;
    constexpr double kTurningPositionDF = 0.0;

    constexpr double kDrivePositionMaxVelocity = 5700.0;     // Rotations per minute.
    constexpr double kDrivePositionMaxAcceleration = 1000.0; // Rotations per minute per second.
    constexpr double kDrivePositionP = 0.004;
    constexpr double kDrivePositionF = 0.0;
    constexpr double kDrivePositionI = 0.0;
    constexpr double kDrivePositionIZ = 0.0;
    constexpr double kDrivePositionIM = 0.0;
    constexpr double kDrivePositionD = 0.0;
    constexpr double kDrivePositionDF = 0.0;

    constexpr double kDriveVelocityMaxVelocity = 5700.0;
    constexpr double kDriveVelocityMaxAcceleration = 1000.0;
    constexpr double kDriveVelocityMaxJerk = 1.0;
    constexpr double kDriveVelocityP = 0.0;
    constexpr double kDriveVelocityF = 0.0;
    constexpr double kDriveVelocityI = 0.0;
    constexpr double kDriveVelocityIZ = 0.0;
    constexpr double kDriveVelocityIM = 0.0;
    constexpr double kDriveVelocityD = 0.0;
    constexpr double kDriveVelocityDF = 0.0;

    constexpr units::degrees_per_second_t kDriveThetaMaxVelocity = 45.0_deg_per_s;
    constexpr units::degrees_per_second_squared_t kDriveThetaMaxAcceleration = 450.0_deg_per_s_sq;
    constexpr double kDriveThetaP = 0.10;
    constexpr double kDriveThetaF = 0.005;
    constexpr double kDriveThetaI = 0.0;
    constexpr double kDriveThetaD = 0.0;
}
namespace nonDrive
{
    constexpr int kShoulderAlignmentOffset = 2047;
    constexpr int kShoulderMotorCanID = 9;
    constexpr int kShoulderEncoderPort = 4;
    constexpr bool kShoulderMotorInverted = true;

    constexpr int kIntakeMotorCanID = 10;
    constexpr bool kIntakeMotorInverted = false;

    constexpr int kTriggerMotorCanID = 11;
    constexpr bool kTriggerMotorInverted = true;

    constexpr int kShooter1MotorCanID = 12;
    constexpr int kShooter2MotorCanID = 13;
    constexpr bool kShooter1MotorInverted = false; // Flipped for 2026 (shooter was spinning backward)
    constexpr bool kShooter2MotorInverted = true;  // Flipped for 2026 (shooter was spinning backward)

    // Shooter (SPARK Flex / Vortex) current limits, applied on the motor
    // controllers.  The smart current limit goes from the stall limit (motor
    // stopped) down to the free limit (motor at speed); the secondary limit is
    // a hard cutoff, as a backup.
    constexpr unsigned int kShooterStallCurrentLimit = 60;  // Amps
    constexpr unsigned int kShooterFreeCurrentLimit = 40;   // Amps
    constexpr double kShooterSecondaryCurrentLimit = 80.0;  // Amps

    // Shooter protection in code: a shooter motor drawing more than this, for
    // longer than this, is stopped until the robot is disabled.  With the
    // limits above in effect, this should never happen.
    constexpr double kShooterTripCurrent = 70.0;            // Amps
    constexpr units::second_t kShooterTripTime = 0.25_s;

    constexpr int kClimber1MotorCanID = 14;
    constexpr int kClimber2MotorCanID = 15;
    constexpr bool kClimber1MotorInverted = true;
    constexpr bool kClimber2MotorInverted = false;

    constexpr int kNoteSensor = 5;

    // 13 on great dane is really loud
    //  dont use 12 unless its for climber code

}
// Button board (Driver Station port 1); its buttons are numbered 13 - 20.
namespace buttonBoard
{
    constexpr int kShooterSpeaker = 13; // Hold: spin up shooter at speaker speed
    constexpr int kShooterFull = 14;    // Hold: spin up shooter at full power
    constexpr int kArmPickup = 15;      // Arm to pickup position
    constexpr int kArmSpeakerClose = 16; // Arm to close speaker shot
    constexpr int kArmSpeakerFar = 17;  // Arm to far speaker shot
    constexpr int kArmStow = 18;        // Arm to stowed position
    constexpr int kAutoSelect = 19;     // On/held when auto starts: Left Side auto (else Right Side)
    constexpr int kAim = 20;            // Hold: aim at the AprilTag target (see vision below)
}

namespace arm
{
    constexpr units::angular_velocity::degrees_per_second_t kShoulderPositionMaxVelocity = 20.0_deg_per_s;
    constexpr units::angular_acceleration::degrees_per_second_squared_t kShoulderPositionMaxAcceleration = 20.0_deg_per_s_sq;

    constexpr units::angle::degree_t kShoulderTolerance = 2.5_deg;

    constexpr double shoulderStaticFeedforward = 0.005;
    constexpr units::angle::degree_t shoulderNegativeStopLimit = +8.4_deg;
    constexpr units::angle::degree_t shoulderPositiveStopLimit = +106.0_deg;
    constexpr units::angle::degree_t shoulderPositiveSlowLimit = +86.0_deg;
    constexpr units::angle::degree_t shoulderNegativeSlowLimit = +30_deg;
    constexpr units::angle::degree_t shoulderNegativeParkLimit = +20.0_deg;
    constexpr units::angle::degree_t shoulderPositiveParkLimit = +98.0_deg;
    constexpr units::angle::degree_t ArmBeginning = +12.0_deg;
    constexpr double shoulderSlowPower = 0.35;
    constexpr double shoulderMaxPower = 0.8;
    constexpr double shoulderParkPower = 0.25;

    constexpr double kShoulderPositionP = 0.018;

    constexpr units::torque::newton_meter_t shoulderMaxTorque = 250.0 * 2.6_Nm;
    // constexpr units::length::meter_t upperArmLength = 28.0_in;
    // constexpr units::mass::kilogram_t pointMass = 2.0_lb;

}

// AprilTag aiming (PhotonVision); hold button board kAim.  The robot turns to
// face the target tag (the driver still drives/strafes), the arm and shooter
// are set from the shot table, and the controller rumbles when ready to shoot
// (then pull the right trigger to feed).
namespace vision
{
    // Camera name, exactly as set in the PhotonVision web interface.
    constexpr const char *kCameraName = "ShooterCamera";

    // Target AprilTag for each alliance (if the alliance is not known, either).
    constexpr int kBlueTargetTag = 7;
    constexpr int kRedTargetTag = 4;

    // A target not seen for this long is treated as lost.
    constexpr units::second_t kTargetTimeout = 0.25_s;

    // Turning toward the tag: turn rate (as a fraction of the maximum turn
    // rate) per degree that the tag is off-center, and the most to use.  If the
    // robot turns AWAY from the tag (e.g. the camera faces backward), change
    // kAimTurnDirection to -1.0.
    constexpr double kAimTurnP = 0.01;
    constexpr double kAimMaxTurn = 0.3;
    constexpr double kAimTurnDirection = 1.0;

    // "Ready" (controller rumbles) when the tag is within this many degrees of
    // center, the arm is in position, and the shooter has been running this
    // long.
    constexpr units::degree_t kAimTolerance = 2.0_deg;
    constexpr units::second_t kShooterSpinUpTime = 1.0_s;

    // The arm angle from the shot table is kept within this range.
    constexpr units::degree_t kMinArmAngle = 12.0_deg;
    constexpr units::degree_t kMaxArmAngle = 86.0_deg;

    // Shot table: for each distance (camera to tag, as shown on the dashboard
    // as Vision/Distance (m)), the arm angle and shooter voltage that score.
    // In between, values are interpolated; outside, the nearest entry is used.
    // Keep the entries in order of increasing distance; add as many as needed.
    //
    // **** PLACEHOLDERS: these use the existing close (36 deg, 7 V) and far
    // (53 deg, 12 V) presets at guessed distances.  Replace them with values
    // measured on the robot: shoot from several distances, find the arm angle
    // and shooter voltage that score, and record Vision/Distance (m).
    struct ShotSetting
    {
        units::meter_t distance;
        units::degree_t armAngle;
        units::volt_t shooterVoltage;
    };

    inline constexpr std::array kShotTable{
        ShotSetting{1.0_m, 36.0_deg, 7.0_V},
        ShotSetting{3.0_m, 53.0_deg, 12.0_V},
    };
}
