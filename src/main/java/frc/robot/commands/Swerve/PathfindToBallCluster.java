package frc.robot.commands.Swerve;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.Set;
import java.util.function.Supplier;

import static frc.robot.Constants.BallVisionConstants.*;

public final class PathfindToBallCluster {

    private PathfindToBallCluster() {}

    /**
     * Build a path that points the intake toward the selected FUEL cluster and
     * stops slightly short so the intake reaches the balls before robot center.
     */
    public static Command build(
            Translation2d target,
            Pose2d fallbackPose,
            Pose2d currentRobotPose) {
        Pose2d targetPose;

        if (target != null) {
            Translation2d robotToTarget = target.minus(currentRobotPose.getTranslation());
            double targetDistance = robotToTarget.getNorm();
            Rotation2d approachHeading = robotToTarget.getAngle();

            double stopDistance = Math.min(
                kBallApproachStopDistanceMeters,
                Math.max(0.0, targetDistance - 0.05)
            );

            Translation2d approachOffset = new Translation2d(
                stopDistance * approachHeading.getCos(),
                stopDistance * approachHeading.getSin()
            );
            Translation2d approachPoint = target.minus(approachOffset);

            targetPose = new Pose2d(approachPoint, approachHeading);
            SmartDashboard.putNumber(
                "BallVision/PathRobotToBallDistanceMeters", targetDistance);
            SmartDashboard.putNumber(
                "BallVision/PathPlannedTravelMeters",
                approachPoint.getDistance(currentRobotPose.getTranslation()));
            SmartDashboard.putNumber("BallVision/PathGoalX", approachPoint.getX());
            SmartDashboard.putNumber("BallVision/PathGoalY", approachPoint.getY());
        } else if (fallbackPose != null) {
            targetPose = fallbackPose;
        } else {
            // Safe test behavior: if no FUEL was detected and no fallback was
            // requested, do not move the drivetrain.
            return Commands.none();
        }

        PathConstraints constraints = new PathConstraints(
            kBallPathfindMaxVelocityMps,
            kBallPathfindMaxAccelMps2,
            Math.toRadians(kBallPathfindMaxAngularVelDeg),
            Math.toRadians(kBallPathfindMaxAngularAccelDeg)
        );

        return AutoBuilder.pathfindToPose(targetPose, constraints, 0.0);
    }

    /**
     * Keep CHASE alive independently of one PathPlanner command's finish result.
     * A completed short path is rebuilt from the current robot pose to the latest
     * filtered target. The caller owns the real collection/lost-target finish gates.
     */
    public static Command buildContinuous(
            Supplier<Translation2d> targetSupplier,
            Supplier<Pose2d> robotPoseSupplier,
            Subsystem drivetrainRequirement) {
        return Commands.repeatingSequence(
            new DeferredCommand(
                () -> build(targetSupplier.get(), null, robotPoseSupplier.get()),
                Set.of(drivetrainRequirement)
            ),
            Commands.waitSeconds(kBallPathReplanDelaySeconds)
        );
    }

    /** Backward-compatible overload for older call sites. */
    public static Command build(
            Translation2d target,
            Pose2d fallbackPose,
            Rotation2d approachHeading) {
        Pose2d targetPose = target != null
            ? new Pose2d(target, approachHeading)
            : fallbackPose;

        PathConstraints constraints = new PathConstraints(
            kPathfindMaxVelocityMps,
            kPathfindMaxAccelMps2,
            Math.toRadians(kPathfindMaxAngularVelDeg),
            Math.toRadians(kPathfindMaxAngularAccelDeg)
        );

        return AutoBuilder.pathfindToPose(targetPose, constraints, 0.0);
    }
}
