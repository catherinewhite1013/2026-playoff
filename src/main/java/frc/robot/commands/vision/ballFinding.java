package frc.robot.commands.vision;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.Swerve.SwerveSubsytem;
import frc.robot.vision.BallClusterFinder;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Consumer;

import static frc.robot.Constants.BallVisionConstants.*;

public class ballFinding extends Command{
    private final SwerveSubsytem swerveSubsytem;
    private final Consumer<Translation2d> resultConsumer; // 掃描結果寫出去給下一個 command 用
    private final double scanDurationSeconds;

    private final List<Translation2d> accumulatedDetections = new ArrayList<>();
    private double startTimestamp;
    private int previousPipelineIndex = kAprilTagPipelineIndex;
    private final boolean sharedCamera;

    public ballFinding(
        SwerveSubsytem swerveSubsytem,
        Consumer<Translation2d> resultConsumer,
        double scanDurationSeconds,
        boolean sharedCamera // true=跟定位共用同一顆相機需切換pipeline, false=獨立相機
    ) {
        this.swerveSubsytem = swerveSubsytem;
        this.resultConsumer = resultConsumer;
        this.scanDurationSeconds = scanDurationSeconds;
        this.sharedCamera = sharedCamera;
        // 注意：這裡沒有 addRequirements(swerveSubsytem)，
        // 因為掃描不需要控制底盤，讓它可以跟移動並行執行 (ParallelCommandGroup)
    }

    @Override
    public void initialize() {
        accumulatedDetections.clear();
        startTimestamp = Timer.getFPGATimestamp();

        if (sharedCamera) {
            LimelightHelpers.setPipelineIndex(kBallLimelightName, kBallDetectorPipelineIndex);
        }
    }

    @Override
    public void execute() {
        List<Translation2d> frameDetections = BallClusterFinder.getVisibleBallFieldPositions(
            kBallLimelightName, swerveSubsytem.getPose());
        accumulatedDetections.addAll(frameDetections);

        SmartDashboard.putNumber("BallVision/AccumulatedDetections", accumulatedDetections.size());
    }

    @Override
    public void end(boolean interrupted) {
        Translation2d cluster = BallClusterFinder.findDensestClusterCenter(accumulatedDetections);

        SmartDashboard.putBoolean("BallVision/ClusterFound", cluster != null);
        if (cluster != null) {
            SmartDashboard.putNumber("BallVision/ClusterX", cluster.getX());
            SmartDashboard.putNumber("BallVision/ClusterY", cluster.getY());
        }

        resultConsumer.accept(cluster); // null 代表沒找到，呼叫端要有 fallback

        if (sharedCamera) {
            LimelightHelpers.setPipelineIndex(kBallLimelightName, previousPipelineIndex);
        }
    }

    @Override
    public boolean isFinished() {
        return Timer.getFPGATimestamp() - startTimestamp >= scanDurationSeconds;
    }
}
