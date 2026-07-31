// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.logging;

import com.studica.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.datalog.BooleanArrayLogEntry;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleArrayLogEntry;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.datalog.IntegerLogEntry;
import edu.wpi.first.util.datalog.StringArrayLogEntry;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.util.datalog.StructArrayLogEntry;
import edu.wpi.first.util.datalog.StructLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Swerve.SwerveSubsytem;

public final class RobotTelemetry {
  private static final String ROOT = "/Telemetry";
  private static final String[] MODULE_ORDER = {
      "FrontLeft", "FrontRight", "BackLeft", "BackRight"
  };
  private static final String[] XBOX_AXIS_ORDER = {
      "LeftX", "LeftY", "RightX", "RightY", "LeftTrigger", "RightTrigger"
  };
  private static final String[] XBOX_BUTTON_ORDER = {
      "A", "B", "X", "Y", "LeftBumper", "RightBumper",
      "Back", "Start", "LeftStick", "RightStick"
  };

  private final SwerveSubsytem swerve;
  private final XboxController driverController;
  private final XboxController operatorController;

  private final StringLogEntry robotMode;
  private final StringLogEntry alliance;
  private final StringLogEntry autonomousCommand;
  private final BooleanLogEntry robotEnabled;
  private final BooleanLogEntry autonomousEnabled;
  private final BooleanLogEntry teleopEnabled;
  private final BooleanLogEntry dsAttached;
  private final BooleanLogEntry fmsAttached;
  private final DoubleLogEntry matchTimeSeconds;

  private final StructLogEntry<Pose2d> estimatedPose;
  private final StructLogEntry<Pose2d> encoderPose;
  private final StructLogEntry<Pose2d> rawVisionPose;
  private final StructLogEntry<Pose2d> acceptedVisionPose;
  private final StructArrayLogEntry<SwerveModulePosition> modulePositions;
  private final StructArrayLogEntry<SwerveModuleState> moduleStates;
  private final DoubleArrayLogEntry moduleAbsoluteAnglesRad;
  private final BooleanLogEntry visionAccepted;
  private final BooleanLogEntry visionTargetVisible;
  private final BooleanLogEntry visionEstimateValid;
  private final BooleanLogEntry visionSeeded;
  private final BooleanLogEntry visionFresh;
  private final DoubleLogEntry visionAgeSeconds;
  private final DoubleLogEntry rawVisionTimestampSeconds;
  private final DoubleLogEntry rawVisionTagCount;
  private final DoubleLogEntry visionTimestampSeconds;
  private final DoubleLogEntry visionTagCount;
  private double lastLoggedRawVisionTimestamp = Double.NaN;

  private final StructLogEntry<Rotation3d> imuRotation;
  private final StructLogEntry<Pose3d> imuIntegratedPose;
  private final DoubleArrayLogEntry imuYawPitchRollDeg;
  private final DoubleLogEntry imuContinuousAngleDeg;
  private final DoubleLogEntry imuRateDegPerSec;
  private final DoubleArrayLogEntry imuRawGyroDegPerSec;
  private final DoubleArrayLogEntry imuRawAccelG;
  private final DoubleArrayLogEntry imuRawMagMicroTesla;
  private final DoubleArrayLogEntry imuWorldLinearAccelG;
  private final DoubleArrayLogEntry imuVelocityMetersPerSec;
  private final DoubleArrayLogEntry imuDisplacementMeters;
  private final BooleanLogEntry imuConnected;
  private final BooleanLogEntry imuCalibrating;
  private final BooleanLogEntry imuMoving;
  private final BooleanLogEntry imuMagneticDisturbance;
  private final BooleanLogEntry imuMagnetometerCalibrated;

  private final DoubleArrayLogEntry driverAxes;
  private final BooleanArrayLogEntry driverButtons;
  private final IntegerLogEntry driverPovDeg;
  private final BooleanLogEntry driverConnected;
  private final StringLogEntry driverName;
  private final DoubleArrayLogEntry operatorAxes;
  private final BooleanArrayLogEntry operatorButtons;
  private final IntegerLogEntry operatorPovDeg;
  private final BooleanLogEntry operatorConnected;
  private final StringLogEntry operatorName;

  public RobotTelemetry(
      SwerveSubsytem swerve,
      XboxController driverController,
      XboxController operatorController) {
    this.swerve = swerve;
    this.driverController = driverController;
    this.operatorController = operatorController;

    DataLog log = DataLogManager.getLog();

    robotMode = new StringLogEntry(log, ROOT + "/Robot/Mode");
    alliance = new StringLogEntry(log, ROOT + "/Robot/Alliance");
    autonomousCommand = new StringLogEntry(log, ROOT + "/Robot/AutonomousCommand");
    robotEnabled = new BooleanLogEntry(log, ROOT + "/Robot/Enabled");
    autonomousEnabled =
        new BooleanLogEntry(log, ROOT + "/Robot/AutonomousEnabled");
    teleopEnabled = new BooleanLogEntry(log, ROOT + "/Robot/TeleopEnabled");
    dsAttached = new BooleanLogEntry(log, ROOT + "/Robot/DSAttached");
    fmsAttached = new BooleanLogEntry(log, ROOT + "/Robot/FMSAttached");
    matchTimeSeconds = new DoubleLogEntry(log, ROOT + "/Robot/MatchTimeSec");

    estimatedPose =
        StructLogEntry.create(log, ROOT + "/Swerve/EstimatedPose", Pose2d.struct);
    encoderPose =
        StructLogEntry.create(log, ROOT + "/Swerve/EncoderPose", Pose2d.struct);
    rawVisionPose =
        StructLogEntry.create(log, ROOT + "/Vision/RawPose", Pose2d.struct);
    acceptedVisionPose =
        StructLogEntry.create(log, ROOT + "/Vision/AcceptedPose", Pose2d.struct);
    modulePositions =
        StructArrayLogEntry.create(
            log, ROOT + "/Swerve/ModulePositions", SwerveModulePosition.struct);
    moduleStates =
        StructArrayLogEntry.create(
            log, ROOT + "/Swerve/ModuleStates", SwerveModuleState.struct);
    moduleAbsoluteAnglesRad =
        new DoubleArrayLogEntry(log, ROOT + "/Swerve/ModuleAbsoluteAnglesRad");
    visionAccepted = new BooleanLogEntry(log, ROOT + "/Vision/AcceptedThisLoop");
    visionTargetVisible = new BooleanLogEntry(log, ROOT + "/Vision/TargetVisible");
    visionEstimateValid =
        new BooleanLogEntry(log, ROOT + "/Vision/NewEstimateValid");
    visionSeeded = new BooleanLogEntry(log, ROOT + "/Vision/HasSeededPose");
    visionFresh = new BooleanLogEntry(log, ROOT + "/Vision/Fresh");
    visionAgeSeconds = new DoubleLogEntry(log, ROOT + "/Vision/AgeSec");
    rawVisionTimestampSeconds =
        new DoubleLogEntry(log, ROOT + "/Vision/RawTimestampSec");
    rawVisionTagCount = new DoubleLogEntry(log, ROOT + "/Vision/RawTagCount");
    visionTimestampSeconds = new DoubleLogEntry(log, ROOT + "/Vision/TimestampSec");
    visionTagCount = new DoubleLogEntry(log, ROOT + "/Vision/TagCount");

    imuRotation =
        StructLogEntry.create(log, ROOT + "/IMU/Rotation3d", Rotation3d.struct);
    imuIntegratedPose =
        StructLogEntry.create(log, ROOT + "/IMU/IntegratedPose3d", Pose3d.struct);
    imuYawPitchRollDeg =
        new DoubleArrayLogEntry(log, ROOT + "/IMU/YawPitchRollDeg");
    imuContinuousAngleDeg =
        new DoubleLogEntry(log, ROOT + "/IMU/ContinuousAngleDeg");
    imuRateDegPerSec =
        new DoubleLogEntry(log, ROOT + "/IMU/YawRateDegPerSec");
    imuRawGyroDegPerSec =
        new DoubleArrayLogEntry(log, ROOT + "/IMU/RawGyroXYZDegPerSec");
    imuRawAccelG =
        new DoubleArrayLogEntry(log, ROOT + "/IMU/RawAccelXYZG");
    imuRawMagMicroTesla =
        new DoubleArrayLogEntry(log, ROOT + "/IMU/RawMagXYZMicroTesla");
    imuWorldLinearAccelG =
        new DoubleArrayLogEntry(log, ROOT + "/IMU/WorldLinearAccelXYZG");
    imuVelocityMetersPerSec =
        new DoubleArrayLogEntry(log, ROOT + "/IMU/VelocityXYZMetersPerSec");
    imuDisplacementMeters =
        new DoubleArrayLogEntry(log, ROOT + "/IMU/DisplacementXYZMeters");
    imuConnected = new BooleanLogEntry(log, ROOT + "/IMU/Connected");
    imuCalibrating = new BooleanLogEntry(log, ROOT + "/IMU/Calibrating");
    imuMoving = new BooleanLogEntry(log, ROOT + "/IMU/Moving");
    imuMagneticDisturbance =
        new BooleanLogEntry(log, ROOT + "/IMU/MagneticDisturbance");
    imuMagnetometerCalibrated =
        new BooleanLogEntry(log, ROOT + "/IMU/MagnetometerCalibrated");

    driverAxes = new DoubleArrayLogEntry(log, ROOT + "/Inputs/Driver/Axes");
    driverButtons = new BooleanArrayLogEntry(log, ROOT + "/Inputs/Driver/Buttons");
    driverPovDeg = new IntegerLogEntry(log, ROOT + "/Inputs/Driver/POVDeg");
    driverConnected = new BooleanLogEntry(log, ROOT + "/Inputs/Driver/Connected");
    driverName = new StringLogEntry(log, ROOT + "/Inputs/Driver/Name");
    operatorAxes = new DoubleArrayLogEntry(log, ROOT + "/Inputs/Operator/Axes");
    operatorButtons = new BooleanArrayLogEntry(log, ROOT + "/Inputs/Operator/Buttons");
    operatorPovDeg = new IntegerLogEntry(log, ROOT + "/Inputs/Operator/POVDeg");
    operatorConnected =
        new BooleanLogEntry(log, ROOT + "/Inputs/Operator/Connected");
    operatorName = new StringLogEntry(log, ROOT + "/Inputs/Operator/Name");

    new StringArrayLogEntry(log, ROOT + "/Schema/SwerveModuleOrder")
        .append(MODULE_ORDER);
    new StringArrayLogEntry(log, ROOT + "/Schema/XboxAxisOrder")
        .append(XBOX_AXIS_ORDER);
    new StringArrayLogEntry(log, ROOT + "/Schema/XboxButtonOrder")
        .append(XBOX_BUTTON_ORDER);
  }

  public void log() {
    logRobotState();
    logSwerve();
    logImu();
    logController(
        driverController,
        driverAxes,
        driverButtons,
        driverPovDeg,
        driverConnected,
        driverName);
    logController(
        operatorController,
        operatorAxes,
        operatorButtons,
        operatorPovDeg,
        operatorConnected,
        operatorName);
  }

  public void logAutonomousCommand(String commandName) {
    autonomousCommand.update(commandName);
  }

  private void logRobotState() {
    robotMode.update(getRobotMode());
    alliance.update(
        DriverStation.getAlliance()
            .map(Enum::name)
            .orElse("Unknown"));
    robotEnabled.update(DriverStation.isEnabled());
    autonomousEnabled.update(DriverStation.isAutonomousEnabled());
    teleopEnabled.update(DriverStation.isTeleopEnabled());
    dsAttached.update(DriverStation.isDSAttached());
    fmsAttached.update(DriverStation.isFMSAttached());
    matchTimeSeconds.append(DriverStation.getMatchTime());
  }

  private void logSwerve() {
    estimatedPose.append(swerve.getPose());
    encoderPose.append(swerve.getEncoderPose());
    modulePositions.append(swerve.getModulePositions());
    moduleStates.append(swerve.getModuleStates());
    moduleAbsoluteAnglesRad.append(swerve.getModuleAbsoluteAnglesRad());
    visionAccepted.update(swerve.wasVisionAccepted());
    visionTargetVisible.update(swerve.didVisionHaveTarget());
    visionEstimateValid.update(swerve.wasVisionEstimateValid());
    visionSeeded.update(swerve.hasVisionSeededPose());
    visionFresh.update(
        swerve.hasFreshVisionPose(DriveConstants.kVisionLockMaxAgeSeconds));
    visionAgeSeconds.append(swerve.getVisionAgeSeconds());

    double rawTimestamp = swerve.getLastRawVisionTimestampSeconds();
    rawVisionTimestampSeconds.update(rawTimestamp);
    rawVisionTagCount.update(swerve.getLastRawVisionTagCount());
    if (rawTimestamp > 0.0
        && Double.compare(rawTimestamp, lastLoggedRawVisionTimestamp) != 0) {
      rawVisionPose.append(swerve.getLatestRawVisionPose());
      lastLoggedRawVisionTimestamp = rawTimestamp;
    }

    visionTimestampSeconds.update(swerve.getLastVisionTimestampSeconds());
    visionTagCount.update(swerve.getVisionTagCount());
    if (swerve.wasVisionAccepted()) {
      acceptedVisionPose.append(swerve.getLatestVisionPose());
    }
  }

  private void logImu() {
    AHRS gyro = swerve.getGyro();
    Rotation3d rotation = gyro.getRotation3d();
    Translation3d displacement =
        new Translation3d(
            gyro.getDisplacementX(),
            gyro.getDisplacementY(),
            gyro.getDisplacementZ());

    imuRotation.append(rotation);
    imuIntegratedPose.append(new Pose3d(displacement, rotation));
    imuYawPitchRollDeg.append(
        new double[] {gyro.getYaw(), gyro.getPitch(), gyro.getRoll()});
    imuContinuousAngleDeg.append(gyro.getAngle());
    imuRateDegPerSec.append(gyro.getRate());
    imuRawGyroDegPerSec.append(
        new double[] {gyro.getRawGyroX(), gyro.getRawGyroY(), gyro.getRawGyroZ()});
    imuRawAccelG.append(
        new double[] {gyro.getRawAccelX(), gyro.getRawAccelY(), gyro.getRawAccelZ()});
    imuRawMagMicroTesla.append(
        new double[] {gyro.getRawMagX(), gyro.getRawMagY(), gyro.getRawMagZ()});
    imuWorldLinearAccelG.append(
        new double[] {
            gyro.getWorldLinearAccelX(),
            gyro.getWorldLinearAccelY(),
            gyro.getWorldLinearAccelZ()
        });
    imuVelocityMetersPerSec.append(
        new double[] {gyro.getVelocityX(), gyro.getVelocityY(), gyro.getVelocityZ()});
    imuDisplacementMeters.append(
        new double[] {
            displacement.getX(),
            displacement.getY(),
            displacement.getZ()
        });
    imuConnected.update(gyro.isConnected());
    imuCalibrating.update(gyro.isCalibrating());
    imuMoving.update(gyro.isMoving());
    imuMagneticDisturbance.update(gyro.isMagneticDisturbance());
    imuMagnetometerCalibrated.update(gyro.isMagnetometerCalibrated());
  }

  private static void logController(
      XboxController controller,
      DoubleArrayLogEntry axes,
      BooleanArrayLogEntry buttons,
      IntegerLogEntry povDeg,
      BooleanLogEntry connected,
      StringLogEntry name) {
    axes.append(
        new double[] {
            controller.getLeftX(),
            controller.getLeftY(),
            controller.getRightX(),
            controller.getRightY(),
            controller.getLeftTriggerAxis(),
            controller.getRightTriggerAxis()
        });
    buttons.update(
        new boolean[] {
            controller.getAButton(),
            controller.getBButton(),
            controller.getXButton(),
            controller.getYButton(),
            controller.getLeftBumperButton(),
            controller.getRightBumperButton(),
            controller.getBackButton(),
            controller.getStartButton(),
            controller.getLeftStickButton(),
            controller.getRightStickButton()
        });
    povDeg.update(controller.getPOV());
    connected.update(controller.isConnected());
    name.update(controller.getName());
  }

  private static String getRobotMode() {
    if (DriverStation.isDisabled()) {
      return "Disabled";
    }
    if (DriverStation.isAutonomous()) {
      return "Autonomous";
    }
    if (DriverStation.isTeleop()) {
      return "Teleop";
    }
    if (DriverStation.isTest()) {
      return "Test";
    }
    return "Unknown";
  }
}
