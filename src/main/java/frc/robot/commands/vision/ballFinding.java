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

import java.util.List;
import java.util.function.Consumer;

import static frc.robot.Constants.BallVisionConstants.*;

public class ballFinding extends Command {
    private final SwerveSubsytem swerveSubsytem;
    private final IntakeSubsystem intakeSubsystem;
    private final Consumer<Translation2d> resultConsumer;
    private final double scanDurationSeconds;
    private final boolean sharedCamera;

    private double startTimestamp;
    private int previousPipelineIndex = kAprilTagPipelineIndex;
    private Translation2d bestClusterCenter;
    private int bestClusterCount;
    private double bestClusterRobotDistance;
    private double firstValidDetectionTimestamp = -1.0;
    private int rawDetectionCount = 0;

    public ballFinding(
            SwerveSubsytem swerveSubsytem,
            IntakeSubsystem intakeSubsystem,
            Consumer<Translation2d> resultConsumer,
            double scanDurationSeconds,
            boolean sharedCamera) {
        this.swerveSubsytem = swerveSubsytem;
        this.intakeSubsystem = intakeSubsystem;
        this.resultConsumer = resultConsumer;
        this.scanDurationSeconds = scanDurationSeconds;
        this.sharedCamera = sharedCamera;

        // This command actively rotates the drivetrain while searching, so it must own
        // the swerve subsystem. The intake is only read for the moving camera transform.
        addRequirements(swerveSubsytem);
    }

    /** Backward-compatible constructor: assumes the camera is at its retracted offset. */
    public ballFinding(
            SwerveSubsytem swerveSubsytem,
            Consumer<Translation2d> resultConsumer,
            double scanDurationSeconds,
            boolean sharedCamera) {
        this(swerveSubsytem, null, resultConsumer, scanDurationSeconds, sharedCamera);
    }

    @Override
    public void initialize() {
        startTimestamp = Timer.getFPGATimestamp();
        bestClusterCenter = null;
        bestClusterCount = 0;
        bestClusterRobotDistance = Double.POSITIVE_INFINITY;
        firstValidDetectionTimestamp = -1.0;
        rawDetectionCount = 0;

        previousPipelineIndex = (int) LimelightHelpers.getCurrentPipelineIndex(kBallLimelightName);

        // IMPORTANT: even a dedicated second camera must be explicitly placed on
        // the FUEL detector pipeline. The old code only did this for sharedCamera.
        LimelightHelpers.setPipelineIndex(kBallLimelightName, kBallDetectorPipelineIndex);

        SmartDashboard.putString("BallVision/CameraName", kBallLimelightName);
        SmartDashboard.putNumber("BallVision/RequestedPipeline", kBallDetectorPipelineIndex);
        SmartDashboard.putBoolean("BallVision/ClusterFound", false);
        SmartDashboard.putBoolean("BallVision/SearchCommandActive", true);
    }

    @Override
    public void execute() {
        Pose2d robotPose = swerveSubsytem.getPose();
        Translation2d cameraRobotOffset = intakeSubsystem != null
            ? intakeSubsystem.getBallCameraRobotOffset()
            : kRobotToBallCamera;
        double cameraHeightMeters = intakeSubsystem != null
            ? intakeSubsystem.getBallCameraHeightMeters()
            : kCameraHeightMeters;

        RawDetection[] rawDetections = LimelightHelpers.getRawDetections(kBallLimelightName);
        rawDetectionCount = rawDetections.length;

        List<Translation2d> frameDetections = BallClusterFinder.getVisibleBallFieldPositions(
            rawDetections,
            robotPose,
            cameraRobotOffset,
            cameraHeightMeters
        );

        ClusterResult frameCluster = BallClusterFinder.findDensestCluster(
            frameDetections,
            robotPose.getTranslation()
        );

        if (frameCluster != null) {
            double robotDistance = frameCluster.center.getDistance(robotPose.getTranslation());

            if (frameCluster.count > bestClusterCount
                    || (frameCluster.count == bestClusterCount
                        && robotDistance < bestClusterRobotDistance)) {
                bestClusterCenter = frameCluster.center;
                bestClusterCount = frameCluster.count;
                bestClusterRobotDistance = robotDistance;
            }
        }

        if (frameCluster != null && firstValidDetectionTimestamp < 0.0) {
            firstValidDetectionTimestamp = Timer.getFPGATimestamp();
        }

        // While searching, slowly rotate the robot so the intake-side camera sweeps the field.
        // Translation stays at zero. Once the short post-detection comparison window ends,
        // isFinished() releases the drivetrain to the PathPlanner command.
        swerveSubsytem.setChassisOutput(
            0.0,
            0.0,
            kBallSearchAngularSpeedRadPerSec,
            false,
            true
        );

        SmartDashboard.putNumber("BallVision/RawDetectionCount", rawDetectionCount);
        SmartDashboard.putNumber("BallVision/FrameDetections", frameDetections.size());
        SmartDashboard.putNumber(
            "BallVision/GeometryRejectedCount",
            Math.max(0, rawDetectionCount - frameDetections.size()));

        RawDetection bestRawDetection = null;
        for (RawDetection detection : rawDetections) {
            if (detection.classId != kBallDetectorClassId) {
                continue;
            }
            if (bestRawDetection == null || detection.ta > bestRawDetection.ta) {
                bestRawDetection = detection;
            }
        }

        if (bestRawDetection != null) {
            SmartDashboard.putNumber("BallVision/RawBestTXNC", bestRawDetection.txnc);
            SmartDashboard.putNumber("BallVision/RawBestTYNC", bestRawDetection.tync);
            SmartDashboard.putNumber("BallVision/RawBestArea", bestRawDetection.ta);
            SmartDashboard.putNumber(
                "BallVision/RawBestEstimatedDistanceMeters",
                BallClusterFinder.estimateHorizontalDistanceMeters(
                    bestRawDetection, cameraHeightMeters));
        }
        SmartDashboard.putString(
            "BallVision/SearchState",
            firstValidDetectionTimestamp < 0.0 ? "SEARCH" : "COMPARE_CLUSTERS"
        );
        SmartDashboard.putNumber("BallVision/BestClusterBallCount", bestClusterCount);
        SmartDashboard.putNumber(
            "BallVision/ActivePipeline",
            LimelightHelpers.getCurrentPipelineIndex(kBallLimelightName)
        );
        SmartDashboard.putBoolean("BallVision/HasRawDetections", rawDetectionCount > 0);
        SmartDashboard.putBoolean("BallVision/HasValidFieldDetections", !frameDetections.isEmpty());
        SmartDashboard.putNumber("BallVision/CameraRobotX", cameraRobotOffset.getX());
        SmartDashboard.putNumber("BallVision/CameraRobotY", cameraRobotOffset.getY());
        SmartDashboard.putNumber("BallVision/CameraHeightMeters", cameraHeightMeters);
        if (intakeSubsystem != null) {
            SmartDashboard.putNumber("BallVision/IntakeEncoderPosition", intakeSubsystem.getExtendPosition());
            SmartDashboard.putNumber("BallVision/IntakeExtensionFraction", intakeSubsystem.getExtendFraction());
        }

        if (bestClusterCenter != null) {
            SmartDashboard.putNumber("BallVision/BestClusterX", bestClusterCenter.getX());
            SmartDashboard.putNumber("BallVision/BestClusterY", bestClusterCenter.getY());
        }
    }

    @Override
    public void end(boolean interrupted) {
        // Stop the active search rotation before the next command takes ownership.
        swerveSubsytem.setChassisOutput(0.0, 0.0, 0.0, false, true);
        SmartDashboard.putString("BallVision/SearchState", interrupted ? "CANCELLED" : "CHASE");
        SmartDashboard.putBoolean("BallVision/SearchCommandActive", false);
        SmartDashboard.putBoolean("BallVision/ClusterFound", bestClusterCenter != null);
        SmartDashboard.putNumber("BallVision/ClusterBallCount", bestClusterCount);

        if (bestClusterCenter != null) {
            SmartDashboard.putNumber("BallVision/ClusterX", bestClusterCenter.getX());
            SmartDashboard.putNumber("BallVision/ClusterY", bestClusterCenter.getY());
        }

        resultConsumer.accept(bestClusterCenter);

        // Dedicated intake camera stays on the FUEL detector pipeline. A shared
        // camera would be restored to its previous pipeline.
        if (sharedCamera) {
            LimelightHelpers.setPipelineIndex(kBallLimelightName, previousPipelineIndex);
        }
    }

    @Override
    public boolean isFinished() {
        // Do NOT time out just because no ball was found. While Driver B is held, this
        // keeps rotating slowly until a valid FUEL cluster appears. After first detection,
        // keep scanning briefly so a larger nearby cluster can win.
        return firstValidDetectionTimestamp >= 0.0
            && Timer.getFPGATimestamp() - firstValidDetectionTimestamp
                >= kBallSearchPostDetectionSeconds;
    }
}