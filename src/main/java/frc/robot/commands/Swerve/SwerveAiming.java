package frc.robot.commands.Swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ShooterConstant;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.Swerve.SwerveSubsytem;

public class SwerveAiming extends Command {

    private final SwerveSubsytem swerveSubsytem;
    private final ShooterSubsystem shooterSubsystem;
    private final int shootMode;

    private double currentRobotAngle = 0;
    private double targetAngle = 0;
    private boolean isRed = false;

    /*
     * Moving-shot ballistic lead
     * ---------------------------
     * The ball inherits the robot's FIELD-relative translation velocity when it
     * leaves the shooter.  Instead of using a fixed look-ahead time, solve the
     * 2-D intercept equation:
     *
     *   |targetVector - robotVelocity * t| = effectiveShotSpeed * t
     *
     * Then aim at:
     *
     *   virtualTarget = realTarget - robotVelocity * t
     *
     * EffectiveShotSpeed is intentionally an empirical horizontal speed.  The
     * current robot code contains distance->RPM maps, but no flywheel diameter,
     * launch angle, or measured ball exit speed, so converting RPM directly into
     * projectile speed would add an unverified physical assumption.
     */
    private static final double kDefaultEffectiveShotSpeedMps = 10.0;
    private static final double kMinEffectiveShotSpeedMps = 2.0;
    private static final double kMaxEffectiveShotSpeedMps = 30.0;

    private static final double kDefaultVelocityFilterAlpha = 0.25;
    private static final double kDefaultLeadScale = 1.50;
    private static final double kDefaultMaxLeadMeters = 1.25;
    private static final double kMaxFlightTimeSeconds = 0.80;

    // Translation speed cap while this aiming/shooting command is active.
    // This is deliberately dashboard-tunable so you can find the fastest speed
    // that still gives repeatable shots without rebuilding code.
    private static final double kDefaultMaxChassisSpeedMps = 0.60;
    private static final double kMinMaxChassisSpeedMps = 0.25;

    private double filteredFieldVx = 0.0;
    private double filteredFieldVy = 0.0;

    public Field2d field = new Field2d();

    private static final PIDController pidController = new PIDController(
        DriveConstants.kPLockHeading,
        DriveConstants.kILockHeading,
        DriveConstants.kDLockHeading
    );

    public SwerveAiming(
        SwerveSubsytem swerveSubsytem,
        ShooterSubsystem shooterSubsystem,
        int shootMode
    ) {
        this.swerveSubsytem = swerveSubsytem;
        this.shooterSubsystem = shooterSubsystem;
        this.shootMode = shootMode;

        pidController.enableContinuousInput(0, 360);
        pidController.setTolerance(DriveConstants.kAimingErrTolerence);
        pidController.setIZone(DriveConstants.kIzLockHeading);

        // Do NOT require the swerve subsystem here.
        // The normal/default drive command keeps X/Y translation alive; this
        // command only supplies the rotation override.
    }

    public static boolean aimIsReady() {
        return Math.abs(pidController.getError()) < DriveConstants.kAimingErrTolerence;
    }

    @Override
    public void initialize() {
        pidController.reset();

        // Start the filter from the currently measured velocity so enabling Aim
        // while already moving does not create a large transient.
        ChassisSpeeds fieldSpeeds = swerveSubsytem.getFieldRelativeSpeeds();
        filteredFieldVx = fieldSpeeds.vxMetersPerSecond;
        filteredFieldVy = fieldSpeeds.vyMetersPerSecond;

        // Keep any values that were already tuned from Shuffleboard/SmartDashboard.
        putDashboardDefault(
            "MovingShot/EffectiveShotSpeedMps",
            kDefaultEffectiveShotSpeedMps);
        putDashboardDefault(
            "MovingShot/VelocityFilterAlpha",
            kDefaultVelocityFilterAlpha);
        // Force the requested moving-shot lead baseline whenever Aim starts.
        // It can still be tuned live from SmartDashboard while this command is active.
        SmartDashboard.putNumber(
            "MovingShot/LeadScale",
            kDefaultLeadScale);
        putDashboardDefault(
            "MovingShot/MaxLeadMeters",
            kDefaultMaxLeadMeters);
        // Force the requested shooting speed cap whenever Aim starts so a stale
        // NetworkTables/SmartDashboard value from an older build cannot leave it fast.
        SmartDashboard.putNumber(
            "MovingShot/MaxChassisSpeedMps",
            kDefaultMaxChassisSpeedMps);

        // Enable the speed cap immediately so the default drive command is
        // limited even before the first execute() ordering completes.
        updateAimingTranslationSpeedLimit();
    }

    private static void putDashboardDefault(String key, double defaultValue) {
        SmartDashboard.putNumber(key, SmartDashboard.getNumber(key, defaultValue));
    }

    @Override
    public void execute() {
        double rotationSpeed = 0;

        // Refresh every loop so the value can be tuned live from SmartDashboard.
        updateAimingTranslationSpeedLimit();

        Pose2d robotPose = swerveSubsytem.getPose();
        currentRobotAngle = robotPose.getRotation().getDegrees() - 180;

        switch (shootMode) {
            case 0: // Hub Shoot + moving-shot ballistic lead
                Translation2d currentTarget = shooterSubsystem.getTargetHubLocation();
                Translation2d compensatedTarget = getMotionCompensatedTarget(currentTarget, robotPose);

                // Aim from the actual shooter position, not blindly from robot center.
                Translation2d shooterPosition = getShooterFieldPosition(robotPose);
                Rotation2d targetFieldAngle = compensatedTarget
                    .minus(shooterPosition)
                    .getAngle();
                targetAngle = targetFieldAngle.getDegrees();

                rotationSpeed = pidController.calculate(currentRobotAngle, targetAngle);
                swerveSubsytem.setAimingRotationOverride(-rotationSpeed);
                field.getObject("hub").setPose(new Pose2d(compensatedTarget, targetFieldAngle));
                break;

            case 1: // Pass: preserve existing behavior, no Hub lead compensation
          var allience = DriverStation.getAlliance();
        
        if (allience.isPresent() && allience.get() == DriverStation.Alliance.Red) {
            targetAngle = 0.0;
                rotationSpeed = pidController.calculate(currentRobotAngle, targetAngle);
                swerveSubsytem.setAimingRotationOverride(-rotationSpeed);
                break;
        } else {
            targetAngle = 180.0;
                rotationSpeed = pidController.calculate(currentRobotAngle, targetAngle);
                swerveSubsytem.setAimingRotationOverride(-rotationSpeed);
                break;
        }

            default:
                swerveSubsytem.clearAimingRotationOverride();
                break;
        }

        // In autonomous, a PathPlanner auto composition can reserve the drivetrain
        // even when the current node is stationary. If no fresh PathPlanner output
        // exists, actively apply 0/0/aim-omega so the robot can rotate in place.
        // While a path is moving, setPathPlannerChassisSpeeds() applies this same
        // aiming omega together with PathPlanner X/Y instead.
        swerveSubsytem.applyAutonomousAimingRotationIfPathPlannerIdle();

        SmartDashboard.putBoolean(
            "MovingShot/AimingOverrideEnabled",
            swerveSubsytem.isAimingRotationOverrideEnabled());
        SmartDashboard.putBoolean(
            "MovingShot/PathPlannerOutputFresh",
            swerveSubsytem.hasFreshPathPlannerOutput());

        SmartDashboard.putNumber("targetAngle", targetAngle);
        SmartDashboard.putNumber("DeltaAngle", pidController.getError());
        SmartDashboard.putNumber("shoot Mode", shootMode);
        SmartDashboard.putBoolean("atSetpoint", pidController.atSetpoint());
        SmartDashboard.putBoolean("aimReady", aimIsReady());
    }


    private void updateAimingTranslationSpeedLimit() {
        double maxChassisSpeedMps = MathUtil.clamp(
            SmartDashboard.getNumber(
                "MovingShot/MaxChassisSpeedMps",
                kDefaultMaxChassisSpeedMps),
            kMinMaxChassisSpeedMps,
            DriveConstants.kPhysicalMaxSpeedMetersPerSecond);

        swerveSubsytem.setAimingTranslationSpeedLimit(maxChassisSpeedMps);

        ChassisSpeeds measuredFieldSpeeds = swerveSubsytem.getFieldRelativeSpeeds();
        SmartDashboard.putNumber(
            "MovingShot/MaxChassisSpeedMpsUsed",
            maxChassisSpeedMps);
        SmartDashboard.putNumber(
            "MovingShot/MeasuredTranslationSpeedMps",
            Math.hypot(
                measuredFieldSpeeds.vxMetersPerSecond,
                measuredFieldSpeeds.vyMetersPerSecond));
    }

    private Translation2d getShooterFieldPosition(Pose2d robotPose) {
        return robotPose.getTranslation().plus(
            ShooterConstant.kRobotToShooter.rotateBy(robotPose.getRotation()));
    }

    /**
     * Compute the virtual target for shooting while translating.
     *
     * This uses the measured field-relative chassis velocity.  If the shooter is
     * later moved away from robot center, the rotational tangential velocity of
     * the shooter is also included automatically.
     */
    private Translation2d getMotionCompensatedTarget(
        Translation2d realTarget,
        Pose2d robotPose
    ) {
        ChassisSpeeds fieldSpeeds = swerveSubsytem.getFieldRelativeSpeeds();

        double alpha = MathUtil.clamp(
            SmartDashboard.getNumber(
                "MovingShot/VelocityFilterAlpha",
                kDefaultVelocityFilterAlpha),
            0.0,
            1.0);

        filteredFieldVx += alpha * (fieldSpeeds.vxMetersPerSecond - filteredFieldVx);
        filteredFieldVy += alpha * (fieldSpeeds.vyMetersPerSecond - filteredFieldVy);

        double leadScale = MathUtil.clamp(
            SmartDashboard.getNumber("MovingShot/LeadScale", kDefaultLeadScale),
            0.0,
            2.0);

        // Center-of-robot translation velocity in field coordinates.
        Translation2d shooterVelocity = new Translation2d(
            filteredFieldVx * leadScale,
            filteredFieldVy * leadScale);

        // Add omega x r for the shooter if it is not exactly at robot center.
        // kRobotToShooter is currently (0,0), so this is presently zero and does
        // not change your existing robot behavior.
        Translation2d shooterOffsetField = ShooterConstant.kRobotToShooter
            .rotateBy(robotPose.getRotation());
        double omega = fieldSpeeds.omegaRadiansPerSecond;
        Translation2d rotationalVelocity = new Translation2d(
            -omega * shooterOffsetField.getY(),
             omega * shooterOffsetField.getX());
        shooterVelocity = shooterVelocity.plus(rotationalVelocity.times(leadScale));

        Translation2d shooterPosition = getShooterFieldPosition(robotPose);
        Translation2d targetVector = realTarget.minus(shooterPosition);

        double effectiveShotSpeed = MathUtil.clamp(
            SmartDashboard.getNumber(
                "MovingShot/EffectiveShotSpeedMps",
                kDefaultEffectiveShotSpeedMps),
            kMinEffectiveShotSpeedMps,
            kMaxEffectiveShotSpeedMps);

        double flightTime = solveInterceptTimeSeconds(
            targetVector,
            shooterVelocity,
            effectiveShotSpeed);

        flightTime = MathUtil.clamp(flightTime, 0.0, kMaxFlightTimeSeconds);

        Translation2d leadOffset = shooterVelocity.times(flightTime);

        // Safety clamp: a bad speed estimate must never create an absurd target.
        double maxLeadMeters = Math.max(
            0.0,
            SmartDashboard.getNumber(
                "MovingShot/MaxLeadMeters",
                kDefaultMaxLeadMeters));

        double rawLeadMeters = leadOffset.getNorm();
        if (rawLeadMeters > maxLeadMeters && rawLeadMeters > 1e-6) {
            leadOffset = leadOffset.times(maxLeadMeters / rawLeadMeters);
        }

        Translation2d compensatedTarget = realTarget.minus(leadOffset);

        double realAngleDeg = targetVector.getAngle().getDegrees();
        double compensatedAngleDeg = compensatedTarget
            .minus(shooterPosition)
            .getAngle()
            .getDegrees();
        double leadAngleDeg = MathUtil.inputModulus(
            compensatedAngleDeg - realAngleDeg,
            -180.0,
            180.0);

        SmartDashboard.putNumber("MovingShot/FieldVx", shooterVelocity.getX());
        SmartDashboard.putNumber("MovingShot/FieldVy", shooterVelocity.getY());
        SmartDashboard.putNumber("MovingShot/RobotSpeed", shooterVelocity.getNorm());
        SmartDashboard.putNumber("MovingShot/TargetDistance", targetVector.getNorm());
        SmartDashboard.putNumber("MovingShot/FlightTimeSecUsed", flightTime);
        SmartDashboard.putNumber("MovingShot/EffectiveShotSpeedMpsUsed", effectiveShotSpeed);
        SmartDashboard.putNumber("MovingShot/LeadX", leadOffset.getX());
        SmartDashboard.putNumber("MovingShot/LeadY", leadOffset.getY());
        SmartDashboard.putNumber("MovingShot/LeadMeters", leadOffset.getNorm());
        SmartDashboard.putNumber("MovingShot/LeadAngleDeg", leadAngleDeg);
        SmartDashboard.putNumber("MovingShot/CompTargetX", compensatedTarget.getX());
        SmartDashboard.putNumber("MovingShot/CompTargetY", compensatedTarget.getY());

        return compensatedTarget;
    }

    /**
     * Solve:
     *
     *   |r - v t| = s t
     *
     * where r is shooter->target, v is shooter field velocity, and s is the
     * effective projectile horizontal speed relative to the shooter.
     *
     * Returns the earliest positive intercept time. If no physical intercept is
     * available (for example a bad tuning value), it safely falls back to
     * distance / shotSpeed.
     */
    private double solveInterceptTimeSeconds(
        Translation2d targetVector,
        Translation2d shooterVelocity,
        double shotSpeedMetersPerSecond
    ) {
        double rx = targetVector.getX();
        double ry = targetVector.getY();
        double vx = shooterVelocity.getX();
        double vy = shooterVelocity.getY();

        double a = vx * vx + vy * vy
            - shotSpeedMetersPerSecond * shotSpeedMetersPerSecond;
        double b = -2.0 * (rx * vx + ry * vy);
        double c = rx * rx + ry * ry;

        double fallback = Math.sqrt(c) / shotSpeedMetersPerSecond;
        final double epsilon = 1e-9;

        if (c < epsilon) {
            return 0.0;
        }

        if (Math.abs(a) < epsilon) {
            if (Math.abs(b) < epsilon) {
                return fallback;
            }

            double t = -c / b;
            return t > 0.0 ? t : fallback;
        }

        double discriminant = b * b - 4.0 * a * c;
        if (discriminant < 0.0) {
            return fallback;
        }

        double sqrtDiscriminant = Math.sqrt(discriminant);
        double t1 = (-b - sqrtDiscriminant) / (2.0 * a);
        double t2 = (-b + sqrtDiscriminant) / (2.0 * a);

        double best = Double.POSITIVE_INFINITY;
        if (t1 > epsilon) {
            best = t1;
        }
        if (t2 > epsilon && t2 < best) {
            best = t2;
        }

        return Double.isFinite(best) ? best : fallback;
    }

    @Override
    public void end(boolean interrupted) {
        // Return both rotation and full translation speed to the normal drive
        // command immediately when the aiming/shooting command ends.
        swerveSubsytem.clearAimingRotationOverride();
        swerveSubsytem.clearAimingTranslationSpeedLimit();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}