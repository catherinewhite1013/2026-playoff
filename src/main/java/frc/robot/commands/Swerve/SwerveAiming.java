package frc.robot.commands.Swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ShooterConstant;
import frc.robot.subsystems.Swerve.SwerveSubsytem;

public class SwerveAiming extends Command {
  private final SwerveSubsytem swerveSubsytem;
  private final int shootMode;

  private final PIDController xController = new PIDController(
      DriveConstants.kPFieldLockTranslation,
      DriveConstants.kIFieldLockTranslation,
      DriveConstants.kDFieldLockTranslation);
  private final PIDController yController = new PIDController(
      DriveConstants.kPFieldLockTranslation,
      DriveConstants.kIFieldLockTranslation,
      DriveConstants.kDFieldLockTranslation);
  private final PIDController headingController = new PIDController(
      DriveConstants.kPLockHeading,
      DriveConstants.kILockHeading,
      DriveConstants.kDLockHeading);
  private final SlewRateLimiter angularSpeedLimiter = new SlewRateLimiter(
      DriveConstants.kFieldLockMaxAngularAccelerationRadiansPerSecondSquared);

  private Pose2d lockedPose = new Pose2d();
  private double lockedHeadingDegrees = 0.0;
  private double previousWrappedHeadingDegrees = 0.0;
  private double unwrappedHeadingDegrees = 0.0;
  private double unwrappedTargetHeadingDegrees = 0.0;
  private boolean lockAcquired = false;

  public SwerveAiming(SwerveSubsytem swerveSubsytem, int shootMode) {
    this.swerveSubsytem = swerveSubsytem;
    this.shootMode = shootMode;

    xController.setTolerance(DriveConstants.kFieldLockTranslationToleranceMeters);
    yController.setTolerance(DriveConstants.kFieldLockTranslationToleranceMeters);
    headingController.setTolerance(DriveConstants.kAimingErrTolerence);

    addRequirements(swerveSubsytem);
  }

  @Override
  public void initialize() {
    xController.reset();
    yController.reset();
    headingController.reset();
    angularSpeedLimiter.reset(0.0);
    previousWrappedHeadingDegrees =
        normalizeDegrees(swerveSubsytem.getRobotRotation().getDegrees());
    unwrappedHeadingDegrees = previousWrappedHeadingDegrees;
    unwrappedTargetHeadingDegrees = unwrappedHeadingDegrees;
    lockAcquired = false;
    SmartDashboard.putBoolean("SwerveAiming/Lock Acquired", false);
  }

  @Override
  public void execute() {
    boolean visionFresh =
        swerveSubsytem.hasFreshVisionPose(DriveConstants.kVisionLockMaxAgeSeconds);

    if (!lockAcquired && swerveSubsytem.hasFieldPose()) {
      // getPose() has already fused botpose_wpiblue, including the configured
      // Limelight robot-space camera offset, with drivetrain odometry.
      lockedPose = swerveSubsytem.getPose();
      lockedHeadingDegrees = normalizeDegrees(getTargetAngleDegrees(lockedPose));
      previousWrappedHeadingDegrees =
          normalizeDegrees(swerveSubsytem.getRobotRotation().getDegrees());
      unwrappedHeadingDegrees = previousWrappedHeadingDegrees;
      unwrappedTargetHeadingDegrees =
          unwrappedHeadingDegrees
              + normalizeDegrees(lockedHeadingDegrees - previousWrappedHeadingDegrees);
      swerveSubsytem.field.getObject("Aim Lock").setPose(
          new Pose2d(lockedPose.getTranslation(), Rotation2d.fromDegrees(lockedHeadingDegrees)));
      swerveSubsytem.field.getObject("Aim Target").setPose(
          new Pose2d(getTargetLocation(), new Rotation2d()));
      lockAcquired = true;
    }

    if (!lockAcquired) {
      swerveSubsytem.stopModules();
      publishTelemetry(visionFresh, new Pose2d(), 0.0, 0.0, 0.0);
      return;
    }

    Pose2d currentPose = swerveSubsytem.getPose();
    double currentHeadingDegrees =
        normalizeDegrees(swerveSubsytem.getRobotRotation().getDegrees());
    updateUnwrappedHeading(currentHeadingDegrees);

    double xSpeedMetersPerSecond =
        xController.calculate(currentPose.getX(), lockedPose.getX());
    double ySpeedMetersPerSecond =
        yController.calculate(currentPose.getY(), lockedPose.getY());
    double angularSpeedRadiansPerSecond =
        headingController.calculate(
            unwrappedHeadingDegrees,
            unwrappedTargetHeadingDegrees);

    if (xController.atSetpoint()) {
      xSpeedMetersPerSecond = 0.0;
    }
    if (yController.atSetpoint()) {
      ySpeedMetersPerSecond = 0.0;
    }
    if (headingController.atSetpoint()) {
      angularSpeedRadiansPerSecond = 0.0;
      angularSpeedLimiter.reset(0.0);
    }

    xSpeedMetersPerSecond = MathUtil.clamp(
        xSpeedMetersPerSecond,
        -DriveConstants.kFieldLockMaxSpeedMetersPerSecond,
        DriveConstants.kFieldLockMaxSpeedMetersPerSecond);
    ySpeedMetersPerSecond = MathUtil.clamp(
        ySpeedMetersPerSecond,
        -DriveConstants.kFieldLockMaxSpeedMetersPerSecond,
        DriveConstants.kFieldLockMaxSpeedMetersPerSecond);
    angularSpeedRadiansPerSecond = MathUtil.clamp(
        angularSpeedRadiansPerSecond,
        -DriveConstants.kFieldLockMaxAngularSpeedRadiansPerSecond,
        DriveConstants.kFieldLockMaxAngularSpeedRadiansPerSecond);
    angularSpeedRadiansPerSecond =
        angularSpeedLimiter.calculate(angularSpeedRadiansPerSecond);

    ChassisSpeeds robotRelativeSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
        xSpeedMetersPerSecond,
        ySpeedMetersPerSecond,
        angularSpeedRadiansPerSecond,
        swerveSubsytem.getRobotRotation());
    swerveSubsytem.setChassisSpeeds(robotRelativeSpeeds);

    publishTelemetry(
        visionFresh,
        currentPose,
        currentHeadingDegrees,
        xSpeedMetersPerSecond,
        ySpeedMetersPerSecond);
    SmartDashboard.putNumber(
        "SwerveAiming/Output Omega RadPerSec",
        angularSpeedRadiansPerSecond);
  }

  private double getTargetAngleDegrees(Pose2d currentPose) {
    if (shootMode == 1) {
      return 180.0;
    }

    return getTargetLocation()
        .minus(currentPose.getTranslation())
        .getAngle()
        .plus(Rotation2d.fromDegrees(ShooterConstant.kShooterFacingOffsetDegrees))
        .getDegrees();
  }

  private Translation2d getTargetLocation() {
    Translation2d target = ShooterConstant.kBlueHubLocation;
    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
      target = ShooterConstant.kRedHubLocation;
    }

    return target;
  }

  private double normalizeDegrees(double angleDegrees) {
    return MathUtil.inputModulus(angleDegrees, -180.0, 180.0);
  }

  private void updateUnwrappedHeading(double currentWrappedHeadingDegrees) {
    double headingDeltaDegrees =
        normalizeDegrees(currentWrappedHeadingDegrees - previousWrappedHeadingDegrees);
    unwrappedHeadingDegrees += headingDeltaDegrees;
    previousWrappedHeadingDegrees = currentWrappedHeadingDegrees;
  }

  private void publishTelemetry(
      boolean visionFresh,
      Pose2d currentPose,
      double currentHeadingDegrees,
      double xSpeedMetersPerSecond,
      double ySpeedMetersPerSecond) {
    SmartDashboard.putBoolean("SwerveAiming/Vision Fresh", visionFresh);
    SmartDashboard.putBoolean(
        "SwerveAiming/Odometry Fallback",
        lockAcquired && !visionFresh);
    SmartDashboard.putBoolean("SwerveAiming/Lock Acquired", lockAcquired);
    SmartDashboard.putNumber("SwerveAiming/Vision Age Sec", swerveSubsytem.getVisionAgeSeconds());
    SmartDashboard.putNumber("SwerveAiming/Locked X", lockedPose.getX());
    SmartDashboard.putNumber("SwerveAiming/Locked Y", lockedPose.getY());
    SmartDashboard.putNumber("SwerveAiming/Error X", lockedPose.getX() - currentPose.getX());
    SmartDashboard.putNumber("SwerveAiming/Error Y", lockedPose.getY() - currentPose.getY());
    SmartDashboard.putNumber(
        "SwerveAiming/Raw Estimated Heading Deg",
        currentPose.getRotation().getDegrees());
    SmartDashboard.putNumber("SwerveAiming/Current Heading Deg", currentHeadingDegrees);
    SmartDashboard.putNumber(
        "SwerveAiming/Current Heading Unwrapped Deg",
        unwrappedHeadingDegrees);
    SmartDashboard.putNumber("SwerveAiming/Target Angle Deg", lockedHeadingDegrees);
    SmartDashboard.putNumber(
        "SwerveAiming/Angle Error Deg",
        unwrappedTargetHeadingDegrees - unwrappedHeadingDegrees);
    SmartDashboard.putNumber("SwerveAiming/Output X Mps", xSpeedMetersPerSecond);
    SmartDashboard.putNumber("SwerveAiming/Output Y Mps", ySpeedMetersPerSecond);
    SmartDashboard.putNumber("SwerveAiming/Shoot Mode", shootMode);
    SmartDashboard.putBoolean(
        "SwerveAiming/At Setpoint",
        lockAcquired
            && xController.atSetpoint()
            && yController.atSetpoint()
            && headingController.atSetpoint());
  }

  @Override
  public void end(boolean interrupted) {
    swerveSubsytem.stopModules();
    SmartDashboard.putBoolean("SwerveAiming/Lock Acquired", false);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
