#pragma once

#include "Constants.h"

#include <frc2/command/SubsystemBase.h>
#include <photon/PhotonCamera.h>
#include <units/angle.h>
#include <units/length.h>
#include <units/time.h>

// AprilTag target tracking (PhotonVision), for aiming the shot.  Each
// Periodic(), this reads the camera's latest results and records the alliance's
// target tag (see vision::kBlueTargetTag/kRedTargetTag), if it was seen.
class VisionSubsystem : public frc2::SubsystemBase
{
public:
  VisionSubsystem() noexcept;

  VisionSubsystem(const VisionSubsystem &) = delete;
  VisionSubsystem &operator=(const VisionSubsystem &) = delete;

  void Periodic() noexcept override;

  // True if the target tag was seen recently (vision::kTargetTimeout).
  bool HasTarget() noexcept;

  // Angle of the target from the center of the camera's view, as reported by
  // PhotonVision, and the distance from the camera to the target (along the
  // floor).  Only meaningful if HasTarget().
  units::degree_t GetYaw() const noexcept { return m_yaw; }
  units::meter_t GetDistance() const noexcept { return m_distance; }

  // Arm angle and shooter voltage for the current distance, from the shot
  // table (vision::kShotTable).
  vision::ShotSetting GetShotSetting() const noexcept { return ShotSettingFor(m_distance); }

  static vision::ShotSetting ShotSettingFor(const units::meter_t distance) noexcept;

  // Shown on the dashboard (Vision/Ready).
  void SetReady(const bool ready) noexcept { m_ready = ready; }

private:
  photon::PhotonCamera m_camera{vision::kCameraName};

  int m_tagId{0};
  units::degree_t m_yaw{0.0};
  units::meter_t m_distance{0.0};
  units::second_t m_lastSeen{-1000.0};
  bool m_ready{false};

  int m_telemetryCounter{0};
};
