package frc.robot.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.RawDetection;

import java.util.ArrayList;
import java.util.List;

import static frc.robot.Constants.BallVisionConstants.*;

public final class BallClusterFinder {

    private BallClusterFinder() {}

    /**
     * Result for one frame's densest visible FUEL cluster.
     */
    public static final class ClusterResult {
        public final Translation2d center;
        public final int count;

        public ClusterResult(Translation2d center, int count) {
            this.center = center;
            this.count = count;
        }
    }

    /**
     * Estimate horizontal camera-to-FUEL distance from one raw neural detection.
     * This diagnostic intentionally does NOT apply kMaxBallDetectionRangeMeters so
     * Dashboard can show whether a raw detection was rejected only because it was
     * estimated beyond the trusted geometry range. Returns NaN for unusable rays.
     */
    public static double estimateHorizontalDistanceMeters(
            RawDetection detection,
            double cameraHeightMeters) {
        if (detection == null || detection.classId != kBallDetectorClassId) {
            return Double.NaN;
        }

        double correctedTyDeg = kCameraUpsideDown ? -detection.tync : detection.tync;
        double rayVerticalAngleRad = Math.toRadians(
            kCameraMountPitchDegrees + correctedTyDeg);
        double denominator = Math.tan(rayVerticalAngleRad);

        if (Math.abs(denominator) < 1e-6) {
            return Double.NaN;
        }

        double distance = (kBallHeightMeters - cameraHeightMeters) / denominator;
        return Double.isFinite(distance) && distance > 0.0 ? distance : Double.NaN;
    }

    /**
     * Convert one neural-detector result into a field-relative FUEL position.
     *
     * The camera is physically rolled 180 degrees. We undo that roll by flipping
     * both txnc and tync before using them. Mount pitch follows the usual math
     * convention: positive = upward, therefore the measured 28 deg downward mount
     * is stored as -28 deg in Constants.
     */
    private static Translation2d detectionToFieldPosition(
            RawDetection detection,
            Pose2d robotPose,
            Translation2d cameraRobotOffset,
            double cameraHeightMeters) {
        if (detection.classId != kBallDetectorClassId) {
            return null;
        }

        double correctedTxDeg = kCameraUpsideDown ? -detection.txnc : detection.txnc;
        double correctedTyDeg = kCameraUpsideDown ? -detection.tync : detection.tync;

        double horizontalDistanceMeters = estimateHorizontalDistanceMeters(
            detection, cameraHeightMeters);
        if (!Double.isFinite(horizontalDistanceMeters)
                || horizontalDistanceMeters > kMaxBallDetectionRangeMeters) {
            return null;
        }

        // Camera X/Y position and height change as the intake extends/retracts.
        // cameraRobotOffset is measured from robot center in robot coordinates and
        // rotated into field coordinates; cameraHeightMeters is used in the ty distance math.
        Translation2d cameraFieldPosition = robotPose.getTranslation().plus(
            cameraRobotOffset.rotateBy(robotPose.getRotation())
        );

        Rotation2d cameraFieldYaw = robotPose.getRotation().plus(
            Rotation2d.fromDegrees(kCameraYawOffsetDegrees)
        );

        // Limelight +tx is to image-right. WPILib field angle is CCW-positive,
        // therefore image-right is a negative field-angle correction.
        double fieldRayAngleRad = cameraFieldYaw.getRadians()
            - Math.toRadians(correctedTxDeg);

        return cameraFieldPosition.plus(new Translation2d(
            horizontalDistanceMeters * Math.cos(fieldRayAngleRad),
            horizontalDistanceMeters * Math.sin(fieldRayAngleRad)
        ));
    }

    /**
     * Read all visible 2026 FUEL detections from the dedicated intake Limelight and
     * return their estimated field positions.
     */
    public static List<Translation2d> getVisibleBallFieldPositions(
            String limelightName,
            Pose2d robotPose,
            Translation2d cameraRobotOffset,
            double cameraHeightMeters) {
        return getVisibleBallFieldPositions(
            LimelightHelpers.getRawDetections(limelightName),
            robotPose,
            cameraRobotOffset,
            cameraHeightMeters);
    }

    /**
     * Same conversion using a caller-supplied raw-detection frame. This lets the
     * command use exactly the same frame for RawDetectionCount and geometry filtering.
     */
    public static List<Translation2d> getVisibleBallFieldPositions(
            RawDetection[] detections,
            Pose2d robotPose,
            Translation2d cameraRobotOffset,
            double cameraHeightMeters) {
        List<Translation2d> positions = new ArrayList<>();

        for (RawDetection detection : detections) {
            Translation2d position = detectionToFieldPosition(
                detection, robotPose, cameraRobotOffset, cameraHeightMeters);
            if (position != null) {
                positions.add(position);
            }
        }

        return positions;
    }

    /**
     * Backward-compatible overload. Assumes the intake/camera is fully retracted.
     */
    public static List<Translation2d> getVisibleBallFieldPositions(
            String limelightName,
            Pose2d robotPose) {
        return getVisibleBallFieldPositions(
            limelightName, robotPose, kRobotToBallCamera, kCameraHeightMeters);
    }

    /**
     * Find the densest visible FUEL region in ONE frame.
     *
     * Priority 1: more FUEL inside kClusterRadiusMeters.
     * Tie-breaker: cluster center closer to the robot.
     *
     * Using one frame for the count avoids counting the same physical FUEL again on
     * every video frame during the scan period.
     */
    public static ClusterResult findDensestCluster(
            List<Translation2d> ballPositions,
            Translation2d robotPosition) {
        if (ballPositions.isEmpty()) {
            return null;
        }

        Translation2d bestCenter = null;
        int bestCount = -1;
        double bestRobotDistance = Double.POSITIVE_INFINITY;

        for (Translation2d candidate : ballPositions) {
            int count = 0;
            double sumX = 0.0;
            double sumY = 0.0;

            for (Translation2d other : ballPositions) {
                if (candidate.getDistance(other) <= kClusterRadiusMeters) {
                    count++;
                    sumX += other.getX();
                    sumY += other.getY();
                }
            }

            Translation2d center = new Translation2d(sumX / count, sumY / count);
            double robotDistance = center.getDistance(robotPosition);

            if (count > bestCount
                    || (count == bestCount && robotDistance < bestRobotDistance)) {
                bestCount = count;
                bestCenter = center;
                bestRobotDistance = robotDistance;
            }
        }

        if (bestCount < kMinDetectionsForValidCluster) {
            return null;
        }

        return new ClusterResult(bestCenter, bestCount);
    }

    /** Backward-compatible helper used by any older code. */
    public static Translation2d findDensestClusterCenter(List<Translation2d> ballPositions) {
        ClusterResult result = findDensestCluster(ballPositions, new Translation2d());
        return result == null ? null : result.center;
    }
}