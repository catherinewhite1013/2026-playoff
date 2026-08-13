// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
// import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  // OIController
  public static class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
    public static final boolean kDriverFieldOriented = true;
    public static final double kDeadband = 0.05;

    public static double deadbandHandler(double value, double deadband) {
      if (Math.abs(value) < deadband) {
        return 0;
      } else if (value > 0) {
        return (value - OIConstants.kDeadband) / (1 - OIConstants.kDeadband);
      } else {
        return (value + OIConstants.kDeadband) / (1 - OIConstants.kDeadband);
      }
    }
  }

  // ID
  public static class IDConstants {
    // Swerve Drive Motor Port
    public static final int kFrontLeftDrivePort = 1;
    public static final int kFrontRightDrivePort = 2;
    public static final int kBackLeftDrivePort = 4;// 1
    public static final int kBackRightDrivePort = 3;// 2

    // Swerve Turning Motor Port
    public static final int kFrontLeftTurnPort = 5;
    public static final int kFrontRightTurnPort = 6;
    public static final int kBackLeftTurnPort = 8;// 5
    public static final int kBackRightTurnPort = 7;// 6

    // Swerve Absolute Encoder Port
    public static final int kFrontLeftDriveAbsoluteEncoderPort = 1;
    public static final int kFrontRightDriveAbsoluteEncoderPort = 2;
    public static final int kBackLeftDriveAbsoluteEncoderPort = 4;
    public static final int kBackRightDriveAbsoluteEncoderPort = 3;

    // Intake
    public static final int kRollerPort = 11;
    public static final int kExtendPort = 10;

    // Storage
    public static final int kStoragePort = 5; //talonfx
    public static final int kShooterFeedPort = 9;

    // Shooter
    public static final int kShooterMainPort = 6; //talonfx
    public static final int kShooterSecondaryPort = 12;
  }

  // SwerveModule
  public static class ModuleConstants {
    public static class MK4i {
      public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
      public static final double kDriveMotorGearRatio = 1.0 / 6.122; // MK3:1.0 / 8.16;
      public static final double kTurningMotorGearRatio = 1 / (150 / 7.0);// MK3: 1 / 8.16
      public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
      public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
      public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
      public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60;

      // Used in working code currently
      public static final double kPTurning = 0.5;

      // These two used for simulation currently
      public static final double kITurning = 0.0;
      public static final double kDTurning = 0.005;
    }
  }

  // SwerveDrive
  public static class DriveConstants {

    // Distance between right and left wheels
    public static final double kTrackWidth = 0.72 - 0.12;

    // Distance between front and back wheels
    public static final double kWheelBase = 0.66 - 0.12;

    // Need to update to correct values, I dont remember the value we set last meet
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2), // FL
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2), // FR
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2), // BL
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2)); // BR

    public static final boolean kFrontLeftDriveMotorReversed = false;
    public static final boolean kFrontRightDriveMotorReversed = false;
    public static final boolean kBackLeftDriveMotorReversed = false;
    public static final boolean kBackRightDriveMotorReversed = false;

    public static final boolean kFrontLeftTurningMotorReversed = true;
    public static final boolean kFrontRightTurningMotorReversed = true;
    public static final boolean kBackLeftTurningMotorReversed = false;
    public static final boolean kBackRightTurningMotorReversed = false;

    public static final boolean kFrontLeftTurningEncoderReversed = false;
    public static final boolean kFrontRightTurningEncoderReversed = false;
    public static final boolean kBackLeftTurningEncoderReversed = true;
    public static final boolean kBackRightTurningEncoderReversed = true;

    public static final double kPhysicalMaxSpeedMetersPerSecond = 5.0;
    public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * Math.PI;

    public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond;
    public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = kPhysicalMaxAngularSpeedRadiansPerSecond;
    public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 5;
    public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 2;

    public static final double kPTheta = 0.04;
    public static final double kITheta = 0.0;
    public static final double kDTheta = 0.0;
    public static final double kIZTheta = 60.0;

    public static final double kMaxDriveMotorTemp = 33.0;

    public static final double kMotorMaxOutput = 1;

//------------------------AIMING-------------------------------- TODO:
    public static final double kPLockHeading = 0.1;
    public static final double kILockHeading = 0.0;
    public static final double kIzLockHeading = 0.0;
    public static final double kDLockHeading = 0.008;
    public static final double kAimingErrTolerence = 5.0;
//--------------------------------------------------------------

    public static final double kPFieldLockTranslation = 1.0;
    public static final double kIFieldLockTranslation = 0.0;
    public static final double kDFieldLockTranslation = 0.0;
    public static final double kFieldLockTranslationToleranceMeters = 0.03;
    public static final double kFieldLockMaxSpeedMetersPerSecond = 0.75;
    public static final double kFieldLockMaxAngularSpeedRadiansPerSecond = 1.5;
    public static final double kFieldLockMaxAngularAccelerationRadiansPerSecondSquared = 4.0;
    public static final double kVisionLockMaxAgeSeconds = 0.35;

    public static final Matrix<N3, N1> kStateStdDevs = VecBuilder.fill(0.1, 0.1, Math.toRadians(2));
    public static final Matrix<N3, N1> kVisionStdDevs = VecBuilder.fill(1.2, 1.2, Math.toRadians(45));
  }

  // Auto
  public static final class AutoConstants {
    public static final double kAutoDriveMaxSpeedMetersPerSecond = DriveConstants.kPhysicalMaxSpeedMetersPerSecond;

    public static final PPHolonomicDriveController pathFollowerConfig = new PPHolonomicDriveController(
        new PIDConstants(5, 0, 0), // Translation constants
        new PIDConstants(3, 0, 0), // Rotation constants
        // // Drive base radius (distance from center to furthest module)
        new Translation2d(DriveConstants.kWheelBase / 2, DriveConstants.kTrackWidth / 2).getNorm());
  }

  public static final class LimelightConstants {
    // Compatibility/test setup: keep the existing shooter-side Limelight NetworkTables name.
    // This must exactly match the physical camera hostname / NT table name.
    public static final String kLimelightName = "limelight-shooter";
    public static final String kLimelightIP = "10.81.69.11";
    
  }

  public static final class BallVisionConstants {
    // Compatibility/test setup: keep the existing intake-side Limelight NT name.
    // The two cameras must still use unique network addresses.
    public static final String kBallLimelightName = "limelight-intake";
    public static final String kLimelight2IP = "10.81.69.12";

    // Pipeline 0 is the 2026 FUEL neural detector on the intake Limelight.
    public static final int kBallDetectorPipelineIndex = 0;
    public static final int kAprilTagPipelineIndex = 0;
    public static final int kBallDetectorClassId = 0;

    // Intake-side camera installation.
    // WPILib / Limelight distance math uses +pitch = upward, so 28 deg downward is -28.
    public static final double kCameraMountPitchDegrees = -28.0;

    // Camera points toward the robot intake. This assumes robot +X / heading 0 points
    // toward the intake. Change this if your pose coordinate convention is different.
    public static final double kCameraYawOffsetDegrees = 0.0;

    // Physical Limelight is rolled 180 degrees (upside-down, Ethernet jack upward).
    // txnc and tync are inverted back into the robot's normal camera coordinate frame.
    public static final boolean kCameraUpsideDown = true;

    // Intake Limelight pose relative to robot center (robot coordinates).
    // +X points toward the intake and +Y points to robot-left.
    // The camera is mounted on the moving intake, so both X and height Z change
    // with intake extension while yaw/pitch remain fixed.
    // Retracted: X = 0.33 m, Y = 0.00 m, Z = 0.38 m.
    // Extended:  X = 0.59 m, Y = 0.00 m, Z = 0.315 m.
    public static final double kBallCameraRetractedXMeters = 0.33;
    public static final double kBallCameraExtensionTravelMeters = 0.26;
    public static final double kBallCameraYMeters = 0.0;
    public static final double kBallCameraRetractedHeightMeters = 0.38;
    public static final double kBallCameraExtendedHeightMeters = 0.315;

    // Backward-compatible fallback height when live IntakeSubsystem geometry is unavailable.
    public static final double kCameraHeightMeters = kBallCameraRetractedHeightMeters;

    // Safe fixed offset used only by backward-compatible helpers when no IntakeSubsystem
    // is available. The live ball-finding command uses IntakeSubsystem.getBallCameraRobotOffset().
    public static final Translation2d kRobotToBallCamera =
        new Translation2d(kBallCameraRetractedXMeters, kBallCameraYMeters);

    // 2026 REBUILT FUEL is 5.91 in nominal diameter; target its center height.
    public static final double kBallDiameterMeters = Units.inchesToMeters(5.91);
    public static final double kBallHeightMeters = kBallDiameterMeters / 2.0;

    // Density analysis: choose the region containing the most visible FUEL first.
    public static final double kClusterRadiusMeters = 0.9;
    public static final int kMinDetectionsForValidCluster = 1;
    public static final double kMaxBallDetectionRangeMeters = 8.0;

    // Stop slightly before the detected cluster so the intake reaches the FUEL first.
    public static final double kBallApproachStopDistanceMeters = 0.35;

    // Pathfinding limits are intentionally unchanged from the existing project.
    // Slow FUEL-search/chase tuning. Hold Driver B to rotate-scan until FUEL is seen,
    // then PathPlanner approaches the selected cluster at a deliberately low speed.
    public static final double kBallSearchAngularSpeedRadPerSec = 0.90;
    public static final double kBallSearchPostDetectionSeconds = 0.25;

    // Continuous Driver-B collection. While chasing, tolerate short detector dropouts.
    // If the selected target stays missing longer than this while the robot is still
    // far away, cancel the stale path and return to SEARCH automatically.
    public static final double kBallTargetLostTimeoutSeconds = 0.35;
    public static final double kBallTargetMatchRadiusMeters = 0.90;
    // Low-pass the field-space target to keep frame-to-frame pose/vision noise from
    // moving the selected FUEL target abruptly. 0 keeps the old point, 1 uses the
    // newest observation directly.
    public static final double kBallTargetPositionFilterAlpha = 0.35;
    // Do not let a one-frame SEARCH -> CHASE handoff gap immediately cancel the path.
    public static final double kBallChaseStartupGraceSeconds = 0.30;
    // Once this close to the old target, finish the last short approach even if the
    // camera loses it because the FUEL has entered the intake/camera blind area.
    public static final double kBallFinishApproachDistanceMeters = 0.70;
    // CHASE completion is based on robot-center distance to the live tracked target,
    // not on PathPlanner command completion. This must be greater than the planned
    // 0.35 m stand-off but smaller than the 0.70 m camera-blind-area allowance.
    public static final double kBallCollectionDistanceMeters = 0.50;
    // Keep drivetrain ownership briefly after reaching the approach point so the
    // intake can pull the FUEL in before the next rotating search begins.
    public static final double kBallCollectSettleSeconds = 0.35;
    // Rebuild a completed short path against the latest filtered target instead of
    // treating one PathPlanner completion as proof that a FUEL was collected.
    public static final double kBallPathReplanDelaySeconds = 0.10;

    // PathPlanner autonomous collection must be finite so the rest of the auto can continue.
    // CollectBallsAuto repeats SEARCH -> CHASE until this timeout expires.
    public static final double kBallAutoCollectTimeoutSeconds = 10.0;

    public static final double kBallPathfindMaxVelocityMps = 1.00;
    public static final double kBallPathfindMaxAccelMps2 = 1.50;
    public static final double kBallPathfindMaxAngularVelDeg = 120.0;
    public static final double kBallPathfindMaxAngularAccelDeg = 240.0;

    // Legacy/general pathfinding limits retained for any other code that still uses them.
    public static final double kPathfindMaxVelocityMps = 3.5;
    public static final double kPathfindMaxAccelMps2 = 4.0;
    public static final double kPathfindMaxAngularVelDeg = 360;
    public static final double kPathfindMaxAngularAccelDeg = 540;
  }

  public static final class IntakeConstants {

    public static final double kExtendFowardPosLimit = 48.5;
    public static final double kExtendReversePosLimit = 0.0;

    public static final double kP = 0.05;
    public static final double kI = 0;
    public static final double kD = 0;
    
    public static final double kExtendMaxOutput = 0.7;
    public static final double kExtendMinOutput = -0.7;
    public static final double kExtendCurrentTripAmps = 60.0;
    public static final double kExtendCurrentTripSeconds = 1.0;

    public static final double kRollerStartMinPos = 40.0;

    public enum ExtendManual{
      kOut(0.2),
      kIn(-0.2),
      kStop(0.0);

      public final double rate;

      private ExtendManual(double rate){
        this.rate = rate;
      }
    }

    public enum ExtendState{
      kExtend(kExtendFowardPosLimit),
      kClose(kExtendReversePosLimit+2);

      public final double position;

      private ExtendState(double position){
        this.position = position;
      }
    }

    public enum RollerAction{
      kGetBall(0.8),
      kStop(0.0),
      kSplitBall(-0.6);

      public final double state;

      private RollerAction(double state){
        this.state = state;
      }
    }
  }

  public static final class StorageConstant {
    
    public enum StorageAction{
      kIn(0.7),
      kStop(0.0),
      kOut(-0.7);

      public final double state;

      private StorageAction(double state){
        this.state = state;
      }
    }

    public enum ShooterFeedAction{
      kIn(0.7),
      kStop(0.0),
      kOut(-0.7);

      public final double state;

      private ShooterFeedAction(double state){
        this.state = state;
      }
    }
  }

  public static final class StatusConstants {
    public static final int kPwmPort = 1;
    public static final int kMainChargeLedCount = 18;
    public static final int kForwardChargeLedCount = 4;
    public static final int kReverseChargeLedCount = 4;
    public static final int kAimLedCount =
        kMainChargeLedCount + kForwardChargeLedCount + kReverseChargeLedCount;
    public static final int kChargeLedCount = 40;
    public static final int kTotalLedCount = kAimLedCount + kChargeLedCount;

    public static final int kRainbowStep = 3;
    public static final int kNormalBrightness = 32;
    public static final int kShootBrightness = 96;
    public static final int kReadyStrobeHalfPeriodTicks = 5;
    public static final int kReadyChargeStepTicks = 5;
  }

  public static final class ShooterConstant {
    
    public static final double kMainFlywheelBeltRatio = 24/36; //Sensor To Mechanism Ratio
    public static final double kShooterFacingOffsetDegrees = 180.0;
    public static final double kMainFlywheelkV = 0.1;
    public static final double kMainFlywheelkS = 0.2;
    public static final double kMainFlywheelkP = 0.5;

    public static final double kSecondaryFlywheelKP = 0.00004;
    public static final double kSecondaryFlywheelKI = 0.0;
    public static final double kSecondaryFlywheelKD = 0.0;
    public static final double kSecondaryFlywheelKV = 0.00204;
    public static final double kSecondaryFlywheelKS = 0.43;

    public static final Translation2d kRobotToShooter = new Translation2d(0.0, 0.0);

    //TODO:
    public static final double[][] kShooterDataMap = {
      {1.6, 2700.0, 1550.0},
      {2.0, 2700.0, 1550.0},// 2 , 2550, 1500
      {2.5, 3000.0, 1550.0},//2.5, 2900, 1500,
      {3.0, 3000.0, 1900.0},
      {3.5, 3400.0, 2000.0},
      {4.0, 3600.0, 2500.0}
    };

    public static final double[][] kShooterDataMapNoIntake = {
      {1.6, 2700.0, 1550.0},
      {2.0, 2700.0, 1550.0},// 2 , 2550, 1500
      {2.5, 3000.0, 1550.0},//2.5, 2900, 1500,
      {3.0, 3000.0, 1900.0},
      {3.0, 3400.0, 2000.0}
    };

    public static final double kMainFlywheelErrTolerence = 300;
    public static final double kSecondaryFlywheelErrTolerence = 300;

    public static final Translation2d kBlueHubLocation = new Translation2d(
        Units.inchesToMeters(181.56),
        Units.inchesToMeters(158.32)); // Welded:Units.inchesToMeters(182.11), Units.inchesToMeters(158.84)
    public static final Translation2d kRedHubLocation = new Translation2d(
        Units.inchesToMeters(468.56),
        Units.inchesToMeters(158.32)); // Welded:Units.inchesToMeters(469.11), Units.inchesToMeters(158.84)
    
    public static final double kRedPassLocation = Units.inchesToMeters(650);
    public static final double kBluePassLocation = Units.inchesToMeters(0);

  }



}
