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

    public static final double kPLockHeading = 0.06;
    public static final double kILockHeading = 0.3;
    public static final double kIzLockHeading = 10;
    public static final double kDLockHeading = 0.0;
    public static final double kAimingErrTolerence = 5.0;

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
    public static final String kLimelightName = "limelight";
    
  }

  public static final class IntakeConstants {

    public static final double kExtendFowardPosLimit = 48.0;
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

  public static final class ShooterConstant {
    
    public static final double kMainFlywheelBeltRatio = 24/36; //Sensor To Mechanism Ratio
    public static final double kShooterFacingOffsetDegrees = 180.0;
    public static final double kMainFlywheelkV = 0.1;
    public static final double kMainFlywheelkS = 0.2;
    public static final double kMainFlywheelkP = 0.5;

    public static final double kSecondaryFlywheelKP = 0.0003;
    public static final double kSecondaryFlywheelKI = 0.0;
    public static final double kSecondaryFlywheelKD = 0.02;
    public static final double kSecondaryFlywheelKV = 0.0032;
    public static final double kSecondaryFlywheelKS = 0.25;

    public static final Translation2d kRobotToShooter = new Translation2d(0.0, 0.0);

    public static final double[][] kShooterDataMap = {
      {1.6, 2700.0, 1500.0},
      {2.0, 2500.0, 1500.0},// 2 , 2550, 1500
      {2.5, 2900.0, 1500.0},//2.5, 2900, 1500,
      {3.0, 2900.0, 1950.0}
    };

    public static final double kMainFlywheelErrTolerence = 300;
    public static final double kSecondaryFlywheelErrTolerence = 300;

    public static final Translation2d kBlueHubLocation = new Translation2d(
        Units.inchesToMeters(181.56),
        Units.inchesToMeters(158.32)); // Welded:Units.inchesToMeters(182.11), Units.inchesToMeters(158.84)
    public static final Translation2d kRedHubLocation = new Translation2d(
        Units.inchesToMeters(468.56),
        Units.inchesToMeters(158.32)); // Welded:Units.inchesToMeters(469.11), Units.inchesToMeters(158.84)
    
    public static final double kRedPassLocation = Units.inchesToMeters(100);
    public static final double kBluePassLocation = Units.inchesToMeters(40);

  }



}
