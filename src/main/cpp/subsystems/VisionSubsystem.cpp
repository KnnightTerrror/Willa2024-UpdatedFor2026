#include "subsystems/VisionSubsystem.h"

#include <frc/DriverStation.h>
#include <frc/Timer.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <units/math.h>

#include <cstdio>
#include <exception>

VisionSubsystem::VisionSubsystem() noexcept
{
    std::printf("Vision: PhotonVision camera \"%s\"; target tag %i (blue) / %i (red).\n",
                vision::kCameraName, vision::kBlueTargetTag, vision::kRedTargetTag);
}

void VisionSubsystem::Periodic() noexcept
{
    // Which tag to look for, based on the alliance (either, if unknown).
    const auto alliance = frc::DriverStation::GetAlliance();
    const bool blue = !alliance || alliance.value() == frc::DriverStation::Alliance::kBlue;
    const bool red = !alliance || alliance.value() == frc::DriverStation::Alliance::kRed;

    try
    {
        // Use the most recent result that includes the target tag.
        for (const auto &result : m_camera.GetAllUnreadResults())
        {
            if (!result.HasTargets())
            {
                continue;
            }

            for (const auto &target : result.GetTargets())
            {
                const int id = target.GetFiducialId();

                if ((blue && id == vision::kBlueTargetTag) || (red && id == vision::kRedTargetTag))
                {
                    const auto translation = target.GetBestCameraToTarget().Translation();

                    m_tagId = id;
                    m_yaw = units::degree_t{target.GetYaw()};
                    m_distance = units::math::hypot(translation.X(), translation.Y());
                    m_lastSeen = frc::Timer::GetFPGATimestamp();
                }
            }
        }
    }
    catch (const std::exception &e)
    {
        std::printf("Vision exception: %s.\n", e.what());
    }

    // Dashboard, 10 times a second.
    if (++m_telemetryCounter < 5)
    {
        return;
    }

    m_telemetryCounter = 0;

    const bool hasTarget = HasTarget();
    const vision::ShotSetting shot = GetShotSetting();

    frc::SmartDashboard::PutBoolean("Vision/Has Target", hasTarget);
    frc::SmartDashboard::PutNumber("Vision/Tag ID", hasTarget ? m_tagId : 0);
    frc::SmartDashboard::PutNumber("Vision/Yaw (deg)", m_yaw.value());
    frc::SmartDashboard::PutNumber("Vision/Distance (m)", m_distance.value());
    frc::SmartDashboard::PutNumber("Vision/Arm Angle (deg)", shot.armAngle.value());
    frc::SmartDashboard::PutNumber("Vision/Shooter Volts", shot.shooterVoltage.value());
    frc::SmartDashboard::PutBoolean("Vision/Ready", m_ready);
}

bool VisionSubsystem::HasTarget() noexcept
{
    return frc::Timer::GetFPGATimestamp() - m_lastSeen < vision::kTargetTimeout;
}

// Linear interpolation in the shot table (nearest entry outside its range),
// with the arm angle kept within the allowed range.
vision::ShotSetting VisionSubsystem::ShotSettingFor(const units::meter_t distance) noexcept
{
    const auto &table = vision::kShotTable;

    vision::ShotSetting shot = table.front();

    if (distance >= table.back().distance)
    {
        shot = table.back();
    }
    else if (distance > table.front().distance)
    {
        for (std::size_t i = 1; i < table.size(); ++i)
        {
            if (distance <= table[i].distance)
            {
                const vision::ShotSetting &low = table[i - 1];
                const vision::ShotSetting &high = table[i];
                const double fraction = ((distance - low.distance) / (high.distance - low.distance)).value();

                shot.distance = distance;
                shot.armAngle = low.armAngle + (high.armAngle - low.armAngle) * fraction;
                shot.shooterVoltage = low.shooterVoltage + (high.shooterVoltage - low.shooterVoltage) * fraction;

                break;
            }
        }
    }

    shot.armAngle = units::math::max(vision::kMinArmAngle, units::math::min(vision::kMaxArmAngle, shot.armAngle));

    return shot;
}
