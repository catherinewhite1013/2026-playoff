// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

// import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
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
    public static final int kFrontLeftDrivePort = 4;
    public static final int kFrontRightDrivePort = 3;
    public static final int kBackLeftDrivePort = 2;// 1
    public static final int kBackRightDrivePort = 1;// 2

    // Swerve Turning Motor Port
    public static final int kFrontLeftTurnPort = 8;
    public static final int kFrontRightTurnPort = 7;
    public static final int kBackLeftTurnPort = 6;// 5
    public static final int kBackRightTurnPort = 5;// 6

    // Intake
    public static final int kRollerPort = 0;
    public static final int kAnglePort = 0;

    // Storage
    public static final int kStoragePort = 0;
    public static final int kShooterFeedPort = 0;

    // Shooter
    public static final int kShooterMainPort = 0;
    public static final int kShooterSecondaryPort = 0;
  }

  // SwerveModule
  public static class ModuleConstants {
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

  // SwerveDrive
  public static class DriveConstants {

    // Distance between right and left wheels
    public static final double kTrackWidth = 0.585;

    // Distance between front and back wheels
    public static final double kWheelBase = kTrackWidth;

    // Need to update to correct values, I dont remember the value we set last meet
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2), // FL
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2), // FR
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2), // BL
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2)); // BR

    public static final boolean kFrontLeftDriveMotorReversed = true;
    public static final boolean kFrontRightDriveMotorReversed = true;
    public static final boolean kBackLeftDriveMotorReversed = true;
    public static final boolean kBackRightDriveMotorReversed = true;

    public static final boolean kFrontLeftTurningMotorReversed = true;
    public static final boolean kFrontRightTurningMotorReversed = true;
    public static final boolean kBackLeftTurningMotorReversed = true;
    public static final boolean kBackRightTurningMotorReversed = true;

    // -------> ABE <-------- //
    public static final int kFrontLeftDriveAbsoluteEncoderPort = 4;
    public static final int kFrontRightDriveAbsoluteEncoderPort = 3;
    public static final int kBackLeftDriveAbsoluteEncoderPort = 2;// 1
    public static final int kBackRightDriveAbsoluteEncoderPort = 1;// 2

    public static final double kPhysicalMaxSpeedMetersPerSecond = 4.5;// 5.5
    public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * Math.PI;

    public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond;
    public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = kPhysicalMaxAngularSpeedRadiansPerSecond;
    public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 2;
    public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 2;// 2

    public static final double kPTheta = 0.04;// 0.012
    public static final double kITheta = 0.0;// 0.01
    public static final double kDTheta = 0.0;// 0.00015
    public static final double kIZTheta = 60.0;// 60.0

    public static final double kMaxDriveMotorTemp = 33.0;

    public static final double kMotorMaxOutput = 1;

    public static final double kPLockHeading = 0.02;
    public static final double kILockHeading = 0.025;
    public static final double kDLockHeading = 0.001;

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
    public static final String kLimelightName = "Front";
    
  }

  public static final class IntakeConstants {

    public static final double kAngleFowardPosLimit = 0.0;
    public static final double kAngleReversePosLimit = 0.0;

    public static final double kP = 0;
    public static final double kI = 0;
    public static final double kD = 0;
    
    public static final double kAngleMaxOutput = 0.5;
    public static final double kAngleMinOutput = -0.5;

    public static final double kRollerStartMinAngle = 0.0;

    public enum AngleManual{
      kOut(0.5),
      kIn(0.5),
      kStop(0.0);

      public final double rate;

      private AngleManual(double rate){
        this.rate = rate;
      }
    }

    public enum AngleState{
      kExtend(0),
      kClose(0);

      public final double position;

      private AngleState(double position){
        this.position = position;
      }
    }

    public enum RollerAction{
      kGetBall(0.6),
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
      kIn(0.8),
      kStop(0.0),
      kOut(0.8);

      public final double state;

      private StorageAction(double state){
        this.state = state;
      }
    }

    public enum ShooterFeedAction{
      kIn(0.8),
      kStop(0.0),
      kOut(0.8);

      public final double state;

      private ShooterFeedAction(double state){
        this.state = state;
      }
    }
  }

  public static final class ShooterConstant {
    
    public static final double kMainFlywheelBeltRatio = 0.0;
    public static final double kMainFlywheelkV = 0.0;
    public static final double kMainFlywheelkS = 0.0;
    public static final double kMainFlywheelkP = 0.0;

    public static final double kSecondaryFlywheelKP = 0.0;
    public static final double kSecondaryFlywheelKI = 0.0;
    public static final double kSecondaryFlywheelKD = 0.0;
    public static final double kSecondaryFlywheelKV = 0.0;
    public static final double kSecondaryFlywheelKS = 0.0;

    public static final Translation2d kRobotToShooter = new Translation2d(0.0, 0.0);

    public static final double[][] kShooterDataMap = {
      {1.0, 0.0, 0.0},
      {2.0, 0.0, 0.0},
    };

    public static final double kMainFlywheelErrTolerence = 100;
    public static final double kSecondaryFlywheelErrTolerence = 50;

    public static final Translation2d kBlueHubLocation = new Translation2d(
        Units.inchesToMeters(181.56),
        Units.inchesToMeters(158.32)); // Welded:Units.inchesToMeters(182.11), Units.inchesToMeters(158.84)
    public static final Translation2d kRedHubLocation = new Translation2d(
        Units.inchesToMeters(468.56),
        Units.inchesToMeters(158.32)); // Welded:Units.inchesToMeters(469.11), Units.inchesToMeters(158.84)

    
  }



}