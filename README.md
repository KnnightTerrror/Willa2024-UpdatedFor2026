# Willa (updated for the 2026 season)
C++ robot code for Willa, Team 7443's 2024 robot, updated to build with the 2026 FRC software (WPILib 2026 and the 2026 vendor libraries).  Swerve drive: SPARK MAX/NEO turning plus SPARK FLEX/Vortex drive and an SRX MAG ENCODER per swerve module (i.e. SDS Swerve Module), with an arm (shoulder), intake, trigger, shooter and climbers; with Test Mode code.

Based on [Jagwires7443/Willa](https://github.com/Jagwires7443/Willa).

## 2026 software versions

| Library | 2024 | 2026 |
| --- | --- | --- |
| WPILib / GradleRIO | 2024.3.2 | 2026.2.1 (Gradle 8.11) |
| REVLib | 2024.2.4 | 2026.0.5 |
| navX | NavX 2024.1.0 (Kauai Labs) | Studica 2026.0.0 |
| PathPlannerLib | 2024.2.8 | removed (no autos used it) |
| CTRE Phoenix 5 / 6 | 5.33.1 / 24.2.0 | removed (not used by this robot) |

Use the **2026 WPILib VS Code** to build and deploy.  Use "WPILib: Manage Vendor Libraries" → "Check for updates (online)" to pick up any newer vendor library releases.

## What changed for 2026

* **REVLib (SPARK MAX / FLEX).**  REV replaced the old one-parameter-at-a-time API (`CANSparkMax`, `SetP()`, `SetSmartCurrentLimit()`, `SetPeriodicFramePeriod()`, `RestoreFactoryDefaults()`, `BurnFlash()`, ...) with config objects applied via `Configure()`.  `infrastructure/SparkMax.cpp` was rewritten for this.  The `SmartMotorBase`/`SmartMotor` interface and the string-keyed `ConfigMap` settings (`"kStatus1"`, `"kSmartCurrentStallLimit"`, `"kP_0"`, ...) are unchanged, so the subsystems and swerve modules work the same way.  The mapping of each key to its REVLib 2026 setting is documented in `infrastructure/SparkMax.h`.  SmartMotion became MAXMotion.
* **navX.**  The navX library is now distributed by Studica: `studica::AHRS` (`<studica/AHRS.h>`), constructed with `NavXComType::kMXP_SPI`.
* **WPILib command-based controllers.**  `CommandXboxController`/`CommandGenericHID` no longer derive from `GenericHID`, so the `JoystickButton`/`POVButton` bindings now use `m_xbox.Start()`, `m_xbox.Back()`, `m_xbox.POVUp()`, `m_xbox.POVDown()`, and the button board's lights/axis counts use `m_buttonBoard.GetHID()`.
* **WPILib deprecations.**  `SwerveModuleState::Optimize()` (now an instance method), `CommandPtr::Schedule()` (now `CommandScheduler::GetInstance().Schedule()`), and the SysId `Config` constructor were updated.
* **PathPlanner.**  Removed.  The 2024 PathPlanner setup was already commented out and no autos used it; the timed autos (button board button 20 selects Left/Right) are unchanged.  To add PathPlanner back, install PathplannerLib 2026 with "WPILib: Manage Vendor Libraries" (this needs an internet connection that can reach `3015rangerrobotics.github.io`), then call `AutoBuilder::configure()` from the drive subsystem; `DriveSubsystem::GetSpeed()`, `Drive(ChassisSpeeds)`, `GetPose()` and `ResetOdometry()` provide what it needs.
* **Test Mode config burning.**  `RobotContainer::TestExit()` is called on entry to every mode; it now only does its work (including saving motor controller config to flash) if Test Mode actually ran.  Before, it tried to burn flash at the start of every Autonomous and Teleop.  REVLib 2026 refuses to do that while enabled.

* **Motor controller settings are reset on startup.**  Every time the code configures a SPARK MAX/FLEX, it first resets the controller to factory defaults, so nothing left over from other code (inversion, follower mode, current limits, ...) can affect it.  The config is also saved to the controller's flash once at startup (as REV recommends), so it comes back correctly after a brownout.

## Before running on the robot

1. Update all SPARK MAX/FLEX firmware with the REV Hardware Client (REVLib 2026 expects current firmware).
2. Update the navX firmware if Studica's tools say it is out of date.
3. Calibrate the swerve module offsets (below).
4. Check that field-oriented drive and the swerve module/arm directions are still correct, before driving at speed.

## Swerve module offsets (calibration)

Each swerve module has an absolute encoder (SRX MAG ENCODER) for the wheel's angle.  The **alignment offset** tells the code which encoder reading means "wheel pointing straight forward".  If the offsets are wrong, the wheels turn to strange angles when the robot is enabled, and the robot will not drive straight.

To calibrate (the robot must be **disabled**):

1. Put the robot up on blocks, so the wheels are off the ground.
2. By hand, point all four wheels **straight forward** (parallel to the robot's sides).  The right-side modules are mirror images of the left-side modules (that is why `kRightDriveMotorInverted` is true and `kLeftDriveMotorInverted` is false), so the **left** wheels' bevel gears must face the **opposite** side from the **right** wheels' bevel gears.  Before calibrating, check the dashboard: if the existing offsets are right, all four **Angle** values will already be close to 0, and there is no need to calibrate.
3. Do **one** of these:
   * hold **Back + Start** on the Xbox controller for 3 seconds, or
   * on the dashboard (SmartDashboard / Shuffleboard / Elastic), set **Swerve/Save Offsets** to true.
4. Check the **Swerve/Calibration** entry on the dashboard (or the Driver Station console): it shows the new offsets, e.g. `Saved: FL +1427, FR -903, RL +384, RR -205`.

The offsets are used right away and saved on the roboRIO (`/home/lvuser/swerve_offsets.txt`), so they are kept across reboots and code deploys.  The console also prints lines you can paste into `Constants.h` (`kFrontLeftAlignmentOffset`, ...), to make them the defaults; the saved file takes priority over `Constants.h` while it exists.

If calibration says an encoder is "not working", check that encoder's wiring (DIO ports 0 - 3: front left, front right, rear left, rear right).

**Checking the result:** enable Teleop (still on blocks) and push the left stick gently forward.  All four wheels should point forward and spin so the robot would drive forward.  If one side spins backward, disable, turn that side's wheels 180°, and calibrate again; if all four spin backward, turn all four 180° and calibrate again.

Note that when the robot is enabled, every module turns to its "forward" angle (0°).  A wheel that was left pointing backward (180°) will turn half a revolution and then stop; this is normal.

### Dashboard values

Under **Swerve/** on the dashboard, for each module: **Encoder OK**, **Raw Position** (encoder reading before the offset), **Offset**, **Angle** (degrees, 0 = forward), **Commanded Angle**, and **Turning Fault**; plus **Gyro Heading**.  Turn a wheel by hand while disabled: its **Angle** should change smoothly and go **up** when the wheel turns counter-clockwise (seen from above).

### Wheels spinning non-stop when enabled

If a module keeps spinning instead of stopping at an angle, its turning motor direction does not match its encoder.  The code now detects this: if a module cannot reach its commanded angle within 2 seconds, it stops that module's turning motor, sets **Turning Fault**, and prints `TURNING FAULT` in the Driver Station console.  The fault clears each time the robot is enabled.  Things to check:

* If **all four** modules fault, flip `physical::kTurningMotorInverted` in `Constants.h`.
* If only one module faults, check that module's encoder (Encoder OK) and that its turning motor CAN ID matches `Constants.h` (CAN IDs 2, 4, 6, 8 are turning; 1, 3, 5, 7 are drive).
* If **Angle** goes **down** when a wheel is turned counter-clockwise by hand, that encoder reads backwards.  Do not just flip `kTurningMotorInverted` in that case (it would stop the spinning, but steering would be mirrored); the encoder direction needs to be reversed in code (`AngleSensor`).

## Controls

### Xbox controller (Driver Station port 0)

| Control | Action |
| --- | --- |
| Left stick | Drive (forward/back, strafe) |
| Right stick (left/right) | Rotate |
| Back | Zero gyro heading and switch to field-oriented drive |
| Start | Toggle slow mode |
| Left trigger (hold) | Intake |
| Left bumper (hold) | Reverse intake |
| Right trigger (hold) | Trigger (feed note into shooter) |
| Right bumper (hold) | Reverse trigger and shooter |
| Y / X (hold) | Climber 1 forward / reverse |
| B / A (hold) | Climber 2 forward / reverse |
| D-pad up / down | Arm to +7° / −7° (both are at/below the arm's lower limit) |
| Back + Start, 3 s, **disabled only** | Calibrate swerve offsets |

The robot starts in robot-oriented drive; press Back (with the robot facing downfield) for field-oriented.

### Button board (Driver Station port 1, buttons 13 - 20)

| Button | Action |
| --- | --- |
| 13 (hold) | Spin up shooter, speaker speed |
| 14 (hold) | Spin up shooter, full power |
| 15 | Arm to pickup (11°) |
| 16 | Arm to close speaker shot (36°) |
| 17 | Arm to far speaker shot (53°) |
| 18 | Arm to stow (17°) |
| 19 | Not used |
| 20 | Auto selector: on/held when Autonomous starts = Left Side auto, otherwise Right Side auto |

To shoot: set the arm angle, hold 13 (or 14) to spin up the shooter, then pull the right trigger to feed.  The button numbers are in the `buttonBoard` namespace in `Constants.h`.

## Features

See [SwerveSensorInterfaceBoard](https://github.com/Jagwires7443/SwerveSensorInterfaceBoard) for details on electrical connections.

Here are some of the features implemented in this code:

* Handling PWM input from SRX MAG ENCODER to derive absolute position;
* Handling SPARK MAX/FLEX motor controllers;
* Profiled PID control of turning position;
* Using turning position to override commanded drive, when modules are not facing in commanded direction;
* Distance and velocity Profiled PID control of drive motors;
* Test Mode includes interactive adjustment of PID settings;
* Error handling for motor controllers, up to being able to test code on a roboRIO with no motor controllers;
* Test Mode uses Shuffleboard to create a tab for each swerve module, and a tab for the overall swerve drive;
* Step-by-step bring up procedure for swerve modules and drive system is documented in block comments (in `src/main/include/infrastructure/SwerveModule.h`);
* Provides several Test Mode routines to automatically have robot drive various fixed test patterns;
* Logic to manage and save configuration of motor controller settings;
* Primitives useful for autonomous driving;
* Integrated with WPILib.

Please see (and/or use) `shuffleboard.json` for suggested Shuffleboard settings.

![Test Mode](TestMode1.PNG)

![Test Mode](TestMode2.PNG)
