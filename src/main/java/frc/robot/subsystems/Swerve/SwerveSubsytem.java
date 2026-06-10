// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Swerve;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.IDConstants;
import frc.robot.Constants.LimelightConstants;
import frc.robot.LimelightHelpers;

public class SwerveSubsytem extends SubsystemBase {
  // Create 4 swerve modules with attributes from constants
  private final SwerveModule frontLeft = new SwerveModule(
      IDConstants.kFrontLeftDrivePort,
      IDConstants.kFrontLeftTurnPort,
      DriveConstants.kFrontLeftDriveAbsoluteEncoderPort,
      DriveConstants.kFrontLeftDriveMotorReversed,
      DriveConstants.kFrontLeftTurningMotorReversed,
      "Front Left");

  private final SwerveModule frontRight = new SwerveModule(
      IDConstants.kFrontRightDrivePort,
      IDConstants.kFrontRightTurnPort,
      DriveConstants.kFrontRightDriveAbsoluteEncoderPort,
      DriveConstants.kFrontRightDriveMotorReversed,
      DriveConstants.kFrontRightTurningMotorReversed,
      "Front Right");

  private final SwerveModule backLeft = new SwerveModule(
      IDConstants.kBackLeftDrivePort,
      IDConstants.kBackLeftTurnPort,
      DriveConstants.kBackLeftDriveAbsoluteEncoderPort,
      DriveConstants.kBackLeftDriveMotorReversed,
      DriveConstants.kBackLeftTurningMotorReversed,
      "Back Left");

  private final SwerveModule backRight = new SwerveModule(
      IDConstants.kBackRightDrivePort,
      IDConstants.kBackRightTurnPort,
      DriveConstants.kBackRightDriveAbsoluteEncoderPort,
      DriveConstants.kBackRightDriveMotorReversed,
      DriveConstants.kBackRightTurningMotorReversed,
      "Back Right");

  private final static AHRS gyro = new AHRS(NavXComType.kMXP_SPI);

  private Field2d field = new Field2d();

  private RobotConfig config;

  private PIDController thetaController;
  public static double heading;

  public double kP = DriveConstants.kPTheta, kI = DriveConstants.kITheta, kD = DriveConstants.kDTheta,
      kIZone = DriveConstants.kIZTheta;

  private boolean isRED = false;

  private final SwerveDrivePoseEstimator poseEstimator;


  // Returns positions of the swerve modules for odometry
  public SwerveModulePosition[] getModulePositions() {

    return (new SwerveModulePosition[] {
        frontLeft.getPosition(),
        frontRight.getPosition(),
        backLeft.getPosition(),
        backRight.getPosition() });

  }



  /* Creates a new SwerveSubsytem. */
  public SwerveSubsytem() {

    resetAllEncoders();

    // Zero navX heading on new thread when robot starts
    new Thread(() -> {
      try {
        Thread.sleep(1000);
        // gyro.calibrate();
        zeroHeading();
      } catch (Exception e) {
      }
    }).start();

    poseEstimator = new SwerveDrivePoseEstimator(
      DriveConstants.kDriveKinematics,
      getOdometryAngle(),
      getModulePositions(),
      new Pose2d(),
      VecBuilder.fill(0.1,0.1,0.1),
      VecBuilder.fill(0.9, 0.9, 0.9)
      );

    // Set default PID values for thetaPID
    thetaController = new PIDController(
    DriveConstants.kPTheta,
    DriveConstants.kITheta,
    DriveConstants.kDTheta);
    thetaController.setIZone(DriveConstants.kIZTheta);
    thetaController.enableContinuousInput(0, 360);


    // Load the RobotConfig from the GUI settings. You should probably
    // store this in your Constants file

    try {
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
    }

    // Configure AutoBuilder last
    AutoBuilder.configure(
        this::getPose, // Robot pose supplier
        this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
        this::getSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        this::setChassisSpeeds, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also
                                // optionally outputs individual module feedforwards
        AutoConstants.pathFollowerConfig,
        config, // The robot configuration
        () -> {
          // Boolean supplier that controls when the path will be mirrored for the red
          // alliance
          // This will flip the path being followed to the red side of the field.
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            isRED = alliance.get() == DriverStation.Alliance.Red;
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        },
        this // Reference to this subsystem to set requirements
    );

    // Set up custom logging to add the current path to a field 2d widget
    PathPlannerLogging.setLogActivePathCallback((poses) -> field.getObject("path").setPoses(poses));

    SmartDashboard.putData("Field", field);
  }

  // Reset gyro heading
  public void zeroHeading() {
    gyro.reset();
    heading = getHeading();
  }

  // Return gyro heading, make sure to read navx docs on this
  public static double getHeading() {
    return gyro.getAngle();
  }

  public double getNormalizedAngle(double angle) {
    return (angle % 360 + 360) % 360; // 0~360
  }

  // Stop all module movement
  public void stopModules() {
    frontLeft.stop();
    frontRight.stop();
    backLeft.stop();
    backRight.stop();
  }

  // Move the swerve modules to the desired SwerveModuleState
  public void setModuleStates(SwerveModuleState[] desiredStates) {

    // Make sure robot rotation is all ways possible by changing other module
    // roation speeds
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);

    frontLeft.setDesiredState(desiredStates[0]);
    frontRight.setDesiredState(desiredStates[1]);
    backLeft.setDesiredState(desiredStates[2]);
    backRight.setDesiredState(desiredStates[3]);
  }

  public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
    // Create module states using array
    SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

    // Set the module state
    setModuleStates(moduleStates);
  }

  public void setChassisOutput(double xSpeed, double ySpeed, double turningAngle) {
    setChassisOutput(xSpeed, ySpeed, turningAngle, false, false);
  }

  public void setChassisOutput(double xSpeed, double ySpeed, double turningAngle, boolean angleFieldRelative) {
    setChassisOutput(xSpeed, ySpeed, turningAngle, angleFieldRelative, false);
  }

  public void setChassisOutput(double xSpeed, double ySpeed, double turningAngle,
      boolean angleFieldRelative, boolean robotRelative) {
    xSpeed *= DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
    ySpeed *= DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;

    double turningSpeed = turningAngle;
    if (angleFieldRelative) {
      // heading = getHeading() - turningAngle;
      turningSpeed = -thetaController.calculate(getNormalizedAngle(getHeading()), turningAngle);
    }

    // System.out.println(getHeading() +" "+heading);

    // turningSpeed *=
    // DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond;

    // double turningSpeed = thetaController.calculate(getHeading(), heading);
    // turningSpeed = Math.abs(turningSpeed) > 0.05 ? turningSpeed : 0.0;
    // turningSpeed *= -DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;
    // turningSpeed = MathUtil.clamp(turningSpeed,
    // -DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond,
    // DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond);

    // Create chassis speeds
    ChassisSpeeds chassisSpeeds;

    if (robotRelative) {
      chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);
    } else {
      if(isRED){
        chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, turningSpeed,
        poseEstimator.getEstimatedPosition().getRotation().plus(Rotation2d.k180deg));
      }else{
        chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, turningSpeed,
        poseEstimator.getEstimatedPosition().getRotation());
      }
    }

    // Set chassis speeds
    setChassisSpeeds(chassisSpeeds);
  }

  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    states[0] = frontLeft.getState();
    states[1] = frontRight.getState();
    states[2] = backLeft.getState();
    states[3] = backRight.getState();
    return states;
  }

  public ChassisSpeeds getSpeeds() {
    return DriveConstants.kDriveKinematics.toChassisSpeeds(getModuleStates());
  }

  // Return robot position caculated by odometer
  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  // Reset odometer to new Pose2d location
  public void resetOdometry(Pose2d pose) {
    poseEstimator.resetPosition(getOdometryAngle(), getModulePositions(), pose);
  }

  // Return an angle from -180 to 180 for robot odometry
  // The commented out method is for if the gyroscope is reversed direction
  public Rotation2d getOdometryAngle() {
    /*
     * double angle = -gyro.getYaw() + 180;
     * if(angle > 180){
     * angle -= 360;
     * }else if(angle < -180){
     * angle += 360;
     * }
     * return Rotation2d.fromDegrees(angle);
     */
    // SmartDashboard.putNumber("Yaw", gyro.getYaw());
    // SmartDashboard.putNumber("Angle", gyro.getAngle());
    // return (Rotation2d.fromDegrees(gyro.getYaw()));
    return Rotation2d.fromDegrees(getRobotDegrees() - 180);
  }

  // Returns an angle from 0 to 360 that is continuous, meaning it loops
  public double getRobotDegrees() {
    double rawValue = -gyro.getAngle() % 360.0;
    if (rawValue < 0.0) {
      return (rawValue + 360.0);
    } else {
      return (rawValue);
    }
  }

  // Reset all swerve encoders
  public void resetAllEncoders() {
    frontLeft.resetEncoders();
    frontRight.resetEncoders();
    backLeft.resetEncoders();
    backRight.resetEncoders();
  }

  public static void copyHeading() {
    heading = getHeading();
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    poseEstimator.update(getOdometryAngle(), getModulePositions());

    LimelightHelpers.SetRobotOrientation(LimelightConstants.kLimelightName, 
    poseEstimator.getEstimatedPosition().getRotation().getDegrees(),
    0, 0, 0, 0, 0
    );

    LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(LimelightConstants.kLimelightName);
    if (LimelightHelpers.getTV(LimelightConstants.kLimelightName)) {
      
      Boolean doRejectUpdate = false;

      if (mt2.tagCount == 0) {
        doRejectUpdate = true;
      }

      if (!doRejectUpdate) {
      poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.7,.7,9999999));
      poseEstimator.addVisionMeasurement(
        mt2.pose,
        mt2.timestampSeconds);
      }
      
    }


    // SwerveModulePosition[] positions = getModulePositions();
    // Rotation2d angle = getOdometryAngle();

    // if (positions == null || angle == null) {
    // System.out.println("Error: Null values in odometry update!");
    // return; // Stop update
    // }

    // // update odometry
    // odometer.update(angle, positions);

    // // Debug
    // SmartDashboard.putNumber("Odometry X", odometer.getPoseMeters().getX());
    // SmartDashboard.putNumber("Odometry Y", odometer.getPoseMeters().getY());
    // SmartDashboard.putNumber("Odometry Angle",
    // odometer.getPoseMeters().getRotation().getDegrees());

    // field.setRobotPose(getPose());

    // Put odometry data on smartdashboard
    SmartDashboard.putNumber("Heading", getHeading());

    // frontLeft.update();
    // frontRight.update();
    // backLeft.update();
    // backRight.update();

  }
}
