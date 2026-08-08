package frc.robot.commands.Swerve;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import static frc.robot.Constants.BallVisionConstants.*;

public final class PathfindToBallCluster {

    private PathfindToBallCluster() {}

    /**
     * 建立一個導航到球堆位置的 Command。
     * targetSupplier 若回傳 null（掃描時沒找到球），fallbackPose 會被使用。
     */
    public static Command build(Translation2d target, Pose2d fallbackPose, Rotation2d approachHeading) {
        Pose2d targetPose = (target != null)
            ? new Pose2d(target, approachHeading)
            : fallbackPose;

        PathConstraints constraints = new PathConstraints(
            kPathfindMaxVelocityMps,
            kPathfindMaxAccelMps2,
            Math.toRadians(kPathfindMaxAngularVelDeg),
            Math.toRadians(kPathfindMaxAngularAccelDeg));

        return AutoBuilder.pathfindToPose(targetPose, constraints, 0.0);
    }
}