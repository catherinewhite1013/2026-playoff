package frc.robot.commands.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.RawDetection;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.Swerve.SwerveSubsytem;
import frc.robot.vision.BallClusterFinder;
import frc.robot.vision.BallClusterFinder.ClusterResult;

import java.util.ArrayList;
import java.util.List;

import static frc.robot.Constants.BallVisionConstants.*;

/**
 * Requirement-free vision observer used while Driver B is held.
 *
 * It never commands the drivetrain or intake. It only keeps a live copy of the
 * currently visible FUEL field positions so a PathPlanner chase can be cancelled
 * when its selected target has genuinely disappeared.
 */
public class BallVisionTracker extends Command {
    private final SwerveSubsytem swerveSubsytem;
    private final IntakeSubsystem intakeSubsystem;

    private List<Translation2d> latestBallPositions = List.of();
    private Translation2d latestClusterCenter = null;
    private int latestClusterCount = 0;
    private double latestValidFrameTimestamp = -1.0;
    private Translation2d selectedTarget = null;
    private double selectedTargetLastSeenTimestamp = -1.0;

    public BallVisionTracker(
            SwerveSubsytem swerveSubsytem,
            IntakeSubsystem intakeSubsystem) {
        this.swerveSubsytem = swerveSubsytem;
        this.intakeSubsystem = intakeSubsystem;
        // Intentionally no addRequirements(): this runs beside SEARCH or PathPlanner.
    }

    @Override
    public void initialize() {
        latestBallPositions = List.of();
        latestClusterCenter = null;
        latestClusterCount = 0;
        latestValidFrameTimestamp = -1.0;
        selectedTarget = null;
        selectedTargetLastSeenTimestamp = -1.0;
        LimelightHelpers.setPipelineIndex(kBallLimelightName, kBallDetectorPipelineIndex);
        SmartDashboard.putBoolean("BallVision/TrackerActive", true);
    }

    @Override
    public void execute() {
        Pose2d robotPose = swerveSubsytem.getPose();
        Translation2d cameraRobotOffset = intakeSubsystem.getBallCameraRobotOffset();
        double cameraHeightMeters = intakeSubsystem.getBallCameraHeightMeters();

        RawDetection[] rawDetections = LimelightHelpers.getRawDetections(kBallLimelightName);
        List<Translation2d> positions = BallClusterFinder.getVisibleBallFieldPositions(
            rawDetections,
            robotPose,
            cameraRobotOffset,
            cameraHeightMeters
        );

        // Copy so callers never observe a list that is being modified in-place.
        latestBallPositions = List.copyOf(new ArrayList<>(positions));

        ClusterResult cluster = BallClusterFinder.findDensestCluster(
            latestBallPositions,
            robotPose.getTranslation()
        );

        if (cluster != null) {
            latestClusterCenter = cluster.center;
            latestClusterCount = cluster.count;
            latestValidFrameTimestamp = Timer.getFPGATimestamp();

            Translation2d matchedTarget = findMatchingTarget(selectedTarget);
            if (matchedTarget != null) {
                selectedTarget = lowPassTarget(selectedTarget, matchedTarget);
                selectedTargetLastSeenTimestamp = latestValidFrameTimestamp;
            }
        } else {
            latestClusterCenter = null;
            latestClusterCount = 0;
        }

        SmartDashboard.putNumber("BallVision/TrackerRawCount", rawDetections.length);
        SmartDashboard.putNumber("BallVision/TrackerValidCount", latestBallPositions.size());
        SmartDashboard.putNumber("BallVision/TrackerClusterCount", latestClusterCount);
        SmartDashboard.putBoolean(
            "BallVision/TrackerHasRecentTarget",
            hasRecentAnyTarget(kBallTargetLostTimeoutSeconds)
        );
        SmartDashboard.putBoolean(
            "BallVision/SelectedTargetRecent",
            isSelectedTargetStillVisible(selectedTarget)
        );
        SmartDashboard.putNumber(
            "BallVision/SelectedTargetAgeSeconds",
            selectedTargetLastSeenTimestamp < 0.0
                ? -1.0
                : Timer.getFPGATimestamp() - selectedTargetLastSeenTimestamp
        );
    }

    /**
     * Begin a new chase using a target that ballFinding just validated.
     *
     * Seeding the timestamp here closes the one-scheduler-cycle gap between the
     * SEARCH command and this independent observer. A single empty Limelight frame
     * at the transition must not cancel the new PathPlanner command.
     */
    public void beginTrackingTarget(Translation2d target) {
        selectedTarget = target;
        selectedTargetLastSeenTimestamp = target == null
            ? -1.0
            : Timer.getFPGATimestamp();
    }

    /** True if any valid FUEL frame has been seen recently. */
    public boolean hasRecentAnyTarget(double maxAgeSeconds) {
        return latestValidFrameTimestamp >= 0.0
            && Timer.getFPGATimestamp() - latestValidFrameTimestamp <= maxAgeSeconds;
    }

    /**
     * True only when a currently/recently visible FUEL lies near the selected field target.
     * This prevents another unrelated ball elsewhere in the image from keeping an old path alive.
     */
    public boolean isSelectedTargetStillVisible(Translation2d selectedTarget) {
        if (selectedTarget == null || this.selectedTarget == null || selectedTargetLastSeenTimestamp < 0.0) {
            return false;
        }

        return Timer.getFPGATimestamp() - selectedTargetLastSeenTimestamp <= kBallTargetLostTimeoutSeconds;
    }

    /** Latest filtered field-space point for the selected FUEL. */
    public Translation2d getTrackedTargetOr(Translation2d fallback) {
        return selectedTarget != null ? selectedTarget : fallback;
    }

    private Translation2d findMatchingTarget(Translation2d target) {
        if (target == null) {
            return null;
        }

        Translation2d closest = null;
        double closestDistance = Double.POSITIVE_INFINITY;
        for (Translation2d ball : latestBallPositions) {
            double distance = ball.getDistance(target);
            if (distance <= kBallTargetMatchRadiusMeters && distance < closestDistance) {
                closest = ball;
                closestDistance = distance;
            }
        }

        // A dense cluster center may sit between several individual FUEL positions.
        if (latestClusterCenter != null) {
            double centerDistance = latestClusterCenter.getDistance(target);
            if (centerDistance <= kBallTargetMatchRadiusMeters
                    && centerDistance < closestDistance) {
                closest = latestClusterCenter;
            }
        }

        return closest;
    }

    private static Translation2d lowPassTarget(
            Translation2d previous,
            Translation2d observation) {
        if (previous == null) {
            return observation;
        }

        double alpha = Math.max(0.0, Math.min(1.0, kBallTargetPositionFilterAlpha));
        return new Translation2d(
            previous.getX() + alpha * (observation.getX() - previous.getX()),
            previous.getY() + alpha * (observation.getY() - previous.getY())
        );
    }

    @Override
    public void end(boolean interrupted) {
        SmartDashboard.putBoolean("BallVision/TrackerActive", false);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
