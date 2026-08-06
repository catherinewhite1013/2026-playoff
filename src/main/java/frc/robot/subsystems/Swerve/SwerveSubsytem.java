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
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.IDConstants;
import frc.robot.Constants.LimelightConstants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.Health.CheckableNavX;
import frc.robot.subsystems.Health.HardwareHealth;


public class SwerveSubsytem extends SubsystemBase {
  // Create 4 swerve modules with attributes from constants
  private final SwerveModule frontLeft = new SwerveModuleMK4i(
      IDConstants.kFrontLeftDrivePort,
      IDConstants.kFrontLeftTurnPort,
      IDConstants.kFrontLeftDriveAbsoluteEncoderPort,
      DriveConstants.kFrontLeftDriveMotorReversed,
      DriveConstants.kFrontLeftTurningMotorReversed,
      DriveConstants.kFrontLeftTurningEncoderReversed,
      "Front Left");

  private final SwerveModule frontRight = new SwerveModuleMK4i(
      IDConstants.kFrontRightDrivePort,
      IDConstants.kFrontRightTurnPort,
      IDConstants.kFrontRightDriveAbsoluteEncoderPort,
      DriveConstants.kFrontRightDriveMotorReversed,
      DriveConstants.kFrontRightTurningMotorReversed,
      DriveConstants.kFrontRightTurningEncoderReversed,
      "Front Right");

  private final SwerveModule backLeft = new SwerveModuleMK4i(
      IDConstants.kBackLeftDrivePort,
      IDConstants.kBackLeftTurnPort,
      IDConstants.kBackLeftDriveAbsoluteEncoderPort,
      DriveConstants.kBackLeftDriveMotorReversed,
      DriveConstants.kBackLeftTurningMotorReversed,
      DriveConstants.kBackLeftTurningEncoderReversed,
      "Back Left");

  private final SwerveModule backRight = new SwerveModuleMK4i(
      IDConstants.kBackRightDrivePort,
      IDConstants.kBackRightTurnPort,
      IDConstants.kBackRightDriveAbsoluteEncoderPort,
      DriveConstants.kBackRightDriveMotorReversed,
      DriveConstants.kBackRightTurningMotorReversed,
      DriveConstants.kBackRightTurningEncoderReversed,
      "Back Right");

  private final static AHRS gyro = new AHRS(NavXComType.kUSB1);

  public Field2d field = new Field2d();

  private RobotConfig config;

  private PIDController thetaController;
  public static double heading;

  public double kP = DriveConstants.kPTheta, kI = DriveConstants.kITheta, kD = DriveConstants.kDTheta,
      kIZone = DriveConstants.kIZTheta;


  // Create odometer for swerve drive
  private final SwerveDrivePoseEstimator poseEstimator;
  private final SwerveDriveOdometry encoderOdometry;

  private Pose2d estPose2d = new Pose2d();
  private Pose2d encoderPose2d = new Pose2d();
  private Pose2d latestVisionPose = new Pose2d();
  private Pose2d latestRawVisionPose = new Pose2d();
  private boolean hasSeededPoseWithVision = false;
  private double lastVisionTimestamp = -1.0;
  private double lastVisionTagCount = 0.0;
  private double lastRawVisionTimestamp = -1.0;
  private double lastRawVisionTagCount = 0.0;
  private boolean lastVisionWasAccepted = false;
  private boolean lastVisionHadTarget = false;
  private boolean lastVisionEstimateWasValid = false;
  private double gyroFieldOffsetDegrees = 0.0;
  private boolean isRedAlliance = false;

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
        getRobotRotation(),
        getModulePositions(),
        new Pose2d(),
        VecBuilder.fill(0.1, 0.1, 0.1),  // 狀態標準差 (X, Y, Theta)
        VecBuilder.fill(0.9, 0.9, 0.9)   // 視覺標準差 (X, Y, Theta)，數值越小越信任視覺
    );
    encoderOdometry = new SwerveDriveOdometry(
        DriveConstants.kDriveKinematics,
        getRobotRotation(),
        getModulePositions(),
        new Pose2d());

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
        this::setPathPlannerChassisSpeeds, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also
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
            // isRedAlliance = alliance.get() == DriverStation.Alliance.Red;
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        },
        this // Reference to this subsystem to set requirements
    );

    // Set up custom logging to add the current path to a field 2d widget
    PathPlannerLogging.setLogActivePathCallback((poses) -> field.getObject("path").setPoses(poses));

    SmartDashboard.putData("Field", field);

    HardwareHealth.getInstance().register(new CheckableNavX(gyro, "Swerve/NavX"));

  }

  // Returns positions of the swerve modules for odometry
  public SwerveModulePosition[] getModulePositions() {

    return (new SwerveModulePosition[] {
        frontLeft.getPosition(),
        frontRight.getPosition(),
        backLeft.getPosition(),
        backRight.getPosition() });

  }

  // Reset gyro heading
  public void zeroHeading() {
    System.out.println("zeroHeading()");

    gyro.reset();

    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
      gyroFieldOffsetDegrees = 180.0 - getRawRobotAngle();
      resetOdometry(new Pose2d(estPose2d.getTranslation(), Rotation2d.kPi));
      poseEstimator.resetRotation(Rotation2d.kPi);
    } else {
      gyroFieldOffsetDegrees = -getRawRobotAngle();
      resetOdometry(new Pose2d(estPose2d.getTranslation(), Rotation2d.kZero));
      poseEstimator.resetRotation(Rotation2d.kZero);
    }

    // for Testing
    // resetOdometry(new Pose2d(1.2, 0.36, Rotation2d.kZero));
  }

  // Returns an angle from 0 to 360 that is continuous, meaning it loops
  private double getRawRobotAngle() { 
    return (-gyro.getAngle() % 360 + 360) % 360;
  }
  

  public double getRobotAngle() {
    return (getRawRobotAngle() + gyroFieldOffsetDegrees + 360) % 360;
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

  private void setPathPlannerChassisSpeeds(ChassisSpeeds chassisSpeeds) {
    // This drivetrain's command frame is rotated 180 degrees from WPILib's
    // robot frame, where +X must point toward the intake.
    setChassisSpeeds(new ChassisSpeeds(
        -chassisSpeeds.vxMetersPerSecond,
        -chassisSpeeds.vyMetersPerSecond,
        -chassisSpeeds.omegaRadiansPerSecond));
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
    // if (angleFieldRelative) {
    // // heading = getHeading() - turningAngle;
    // turningSpeed = -thetaController.calculate(getNormalizedAngle(getHeading()),
    // turningAngle);
    // }

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

      var alliance = DriverStation.getAlliance();
      if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
        xSpeed = -xSpeed;
        ySpeed = -ySpeed;
      }

      chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
          xSpeed, ySpeed, turningSpeed,
          estPose2d.getRotation());
      // odometer.getPoseMeters().getRotation());
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
    // return odometer.getPoseMeters();
    return estPose2d;
  }

  public Pose2d getLatestVisionPose() {
    return latestVisionPose;
  }

  public Pose2d getLatestRawVisionPose() {
    return latestRawVisionPose;
  }

  public Pose2d getEncoderPose() {
    return encoderPose2d;
  }

  public boolean wasVisionAccepted() {
    return lastVisionWasAccepted;
  }

  public boolean hasVisionSeededPose() {
    return hasSeededPoseWithVision;
  }

  public double getVisionTagCount() {
    return lastVisionTagCount;
  }

  public double getLastVisionTimestampSeconds() {
    return lastVisionTimestamp;
  }

  public double getLastRawVisionTimestampSeconds() {
    return lastRawVisionTimestamp;
  }

  public double getLastRawVisionTagCount() {
    return lastRawVisionTagCount;
  }

  public boolean didVisionHaveTarget() {
    return lastVisionHadTarget;
  }

  public boolean wasVisionEstimateValid() {
    return lastVisionEstimateWasValid;
  }

  public double[] getModuleAbsoluteAnglesRad() {
    return new double[] {
        frontLeft.getAbsoluteEncoderRad(),
        frontRight.getAbsoluteEncoderRad(),
        backLeft.getAbsoluteEncoderRad(),
        backRight.getAbsoluteEncoderRad()
    };
  }

  public double getVisionAgeSeconds() {
    if (lastVisionTimestamp <= 0.0) {
      return -1.0;
    }
    return Math.max(0.0, Timer.getFPGATimestamp() - lastVisionTimestamp);
  }

  public boolean hasFreshVisionPose(double maxAgeSeconds) {
    double visionAgeSeconds = getVisionAgeSeconds();
    return hasSeededPoseWithVision
        && lastVisionTagCount > 0.0
        && visionAgeSeconds >= 0.0
        && visionAgeSeconds <= maxAgeSeconds;
  }

  public boolean hasFieldPose() {
    return hasSeededPoseWithVision;
  }

  // Reset odometer to new Pose2d location
  public void resetOdometry(Pose2d pose) {
    poseEstimator.resetPosition(getRobotRotation(), getModulePositions(), pose);
    encoderOdometry.resetPosition(getRobotRotation(), getModulePositions(), pose);
  }

  // Return an angle from -180 to 180 for robot odometry
  // The commented out method is for if the gyroscope is reversed direction
  public Rotation2d getRobotRotation() {
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
    return Rotation2d.fromDegrees(getRobotAngle());
  }

  // Reset all swerve encoders
  public void resetAllEncoders() {
    frontLeft.resetEncoders();
    frontRight.resetEncoders();
    backLeft.resetEncoders();
    backRight.resetEncoders();
  }

  public AHRS getGyro() {
    return gyro;
  }
  

  private LimelightHelpers.PoseEstimate getBestLimelightPoseEstimate() { 
    LimelightHelpers.PoseEstimate mt1 =
        LimelightHelpers.getBotPoseEstimate_wpiBlue(LimelightConstants.kLimelightName);
    if (!hasSeededPoseWithVision && isValidVisionEstimate(mt1)) {
      //System.out.println("mt1");
      return mt1;
    }

    LimelightHelpers.PoseEstimate mt2 =
        LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(LimelightConstants.kLimelightName);
    if (isValidVisionEstimate(mt2)) {
      //System.out.println("mt2");
      return mt2;
    }
    //System.out.println("mt1 (both not valid)");
    return mt1;
  }

  private boolean isValidVisionEstimate(LimelightHelpers.PoseEstimate estimate) {
    if (estimate == null || estimate.tagCount <= 0) {
      return false;
    }

    Pose2d pose = estimate.pose;
    if (pose == null || estimate.timestampSeconds <= 0.0) {
      return false;
    }

    if (estimate.timestampSeconds <= lastVisionTimestamp) {
      return false;
    }

    double x = pose.getX();
    double y = pose.getY();
    if (Double.isNaN(x) || Double.isNaN(y)) {
      return false;
    }

    return !(Math.abs(x) < 0.01 && Math.abs(y) < 0.01);
  }

  private void addVisionMeasurement(LimelightHelpers.PoseEstimate estimate) {
    latestVisionPose = estimate.pose;
    lastVisionTimestamp = estimate.timestampSeconds;
    lastVisionTagCount = estimate.tagCount;

    if (!hasSeededPoseWithVision) {
      gyroFieldOffsetDegrees =
          latestVisionPose.getRotation().getDegrees() - getRawRobotAngle(); 
      poseEstimator.resetPosition(getRobotRotation(), getModulePositions(), latestVisionPose);
      hasSeededPoseWithVision = true;
      return;
    }

    poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(0.5, 0.5, 9999999));  //TODO:
    poseEstimator.addVisionMeasurement(latestVisionPose, estimate.timestampSeconds);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    poseEstimator.update(getRobotRotation(), getModulePositions());
    encoderPose2d = encoderOdometry.update(getRobotRotation(), getModulePositions());


    LimelightHelpers.SetRobotOrientation(
        LimelightConstants.kLimelightName, 
        getRobotRotation().getDegrees(),
        0, 0, 0, 0, 0);

    lastVisionWasAccepted = false;
    LimelightHelpers.PoseEstimate visionEstimate = getBestLimelightPoseEstimate();
    lastVisionHadTarget = LimelightHelpers.getTV(LimelightConstants.kLimelightName);
    lastVisionEstimateWasValid = isValidVisionEstimate(visionEstimate);

    if (visionEstimate != null) {
      lastRawVisionTimestamp = visionEstimate.timestampSeconds;
      lastRawVisionTagCount = visionEstimate.tagCount;
      if (visionEstimate.pose != null) {
        latestRawVisionPose = visionEstimate.pose;
      }
    }

    if (lastVisionHadTarget && lastVisionEstimateWasValid) {
      addVisionMeasurement(visionEstimate);
      lastVisionWasAccepted = true;
    }

    estPose2d = poseEstimator.getEstimatedPosition();
    field.setRobotPose(estPose2d);
    field.getObject("Encoder").setPose(encoderPose2d);
    field.getObject("Vision").setPose(latestVisionPose);
    field.getObject("Estimated").setPose(estPose2d);

    

    // SwerveModulePosition[] positions = getModulePositions();
    // Rotation2d angle = getRobotRotation();

    // if (positions == null || angle == null) {
    // System.out.println("Error: Null values in odometry update!");
    // return; // Stop update
    // }

    // // update odometry
    // odometer.update(angle, positions);

    // Debug
    SmartDashboard.putNumber("Estimated X", estPose2d.getX());
    SmartDashboard.putNumber("Estimated Y", estPose2d.getY());
    SmartDashboard.putNumber("Estimated Angle", estPose2d.getRotation().getDegrees());
    SmartDashboard.putNumber("Encoder X", encoderPose2d.getX());
    SmartDashboard.putNumber("Encoder Y", encoderPose2d.getY());
    SmartDashboard.putNumber("Vision X", latestVisionPose.getX());
    SmartDashboard.putNumber("Vision Y", latestVisionPose.getY());
    SmartDashboard.putNumber("Vision Tag Count", lastVisionTagCount);
    SmartDashboard.putBoolean("Vision Seeded Pose", hasSeededPoseWithVision);
    SmartDashboard.putBoolean("Vision Accepted", lastVisionWasAccepted);

    // frontLeft.update();
    // frontRight.update();
    // backLeft.update();
    // backRight.update();

    frontLeft.printInfo();
    frontRight.printInfo();
    backLeft.printInfo();
    backRight.printInfo();

    SmartDashboard.putNumber("Heading", getRobotAngle());
  }

  public ChassisSpeeds getFieldRelativeSpeeds() {
    return ChassisSpeeds.fromRobotRelativeSpeeds(
        getSpeeds(),
        getRobotRotation());
  }

}
