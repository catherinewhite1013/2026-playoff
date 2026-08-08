package frc.robot.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.RawDetection;

import java.util.ArrayList;
import java.util.List;

import static frc.robot.Constants.BallVisionConstants.*;

public class BallClusterFinder {

    /**
     * 將單次原始偵測，依機器人當下 pose 轉換成場地座標。
     * distance 估算公式來自 Limelight 官方文件：
     * d = (h_target - h_camera) / tan(cameraAngle + ty)
     */
    private static Translation2d detectionToFieldPosition(RawDetection d, Pose2d robotPose) {
        double tyRad = Math.toRadians(d.tync);
        double cameraAngleRad = Math.toRadians(kCameraMountAngleDegrees);

        double heightDiff = kBallHeightMeters - kCameraHeightMeters;
        double denom = Math.tan(cameraAngleRad + tyRad);

        if (Math.abs(denom) < 1e-6) {
            return null; // 幾乎水平角度，距離估算不可靠
        }

        double distanceMeters = heightDiff / denom;
        if (distanceMeters <= 0 || distanceMeters > kMaxBallDetectionRangeMeters) {
            return null; // 過濾掉不合理或超出可信範圍的偵測
        }

        double txRad = Math.toRadians(d.txnc);
        // 機器人朝向 + 相機水平偏角 = 場地絕對角度
        double fieldAngleRad = robotPose.getRotation().getRadians() - txRad;

        double ballX = robotPose.getX() + distanceMeters * Math.cos(fieldAngleRad);
        double ballY = robotPose.getY() + distanceMeters * Math.sin(fieldAngleRad);

        return new Translation2d(ballX, ballY);
    }

    /**
     * 掃一次目前影像，回傳所有可信的場地座標球位置。
     */
    public static List<Translation2d> getVisibleBallFieldPositions(String limelightName, Pose2d robotPose) {
        RawDetection[] detections = LimelightHelpers.getRawDetections(limelightName);
        List<Translation2d> positions = new ArrayList<>();

        for (RawDetection d : detections) {
            Translation2d pos = detectionToFieldPosition(d, robotPose);
            if (pos != null) {
                positions.add(pos);
            }
        }
        return positions;
    }

    /**
     * 在一批場地座標球位置中，找出密度最高的群集中心。
     * 簡單 O(n^2) 半徑法，球數量通常不多（幾顆到十幾顆），效能足夠。
     */
    public static Translation2d findDensestClusterCenter(List<Translation2d> ballPositions) {
        if (ballPositions.isEmpty()) {
            return null;
        }

        Translation2d bestCenter = null;
        int bestCount = -1;

        for (Translation2d candidate : ballPositions) {
            int count = 0;
            double sumX = 0, sumY = 0;

            for (Translation2d other : ballPositions) {
                if (candidate.getDistance(other) <= kClusterRadiusMeters) {
                    count++;
                    sumX += other.getX();
                    sumY += other.getY();
                }
            }

            if (count > bestCount) {
                bestCount = count;
                bestCenter = new Translation2d(sumX / count, sumY / count);
            }
        }

        if (bestCount < kMinDetectionsForValidCluster) {
            return null; // 沒有足夠可信的群集，避免誤導航
        }

        return bestCenter;
    }
}