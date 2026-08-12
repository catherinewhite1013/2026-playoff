package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IDConstants;
import frc.robot.Constants.IntakeConstants.ExtendManual;
import frc.robot.Constants.IntakeConstants.ExtendState;
import frc.robot.Constants.IntakeConstants.RollerAction;
import frc.robot.subsystems.Health.CheckableSpark;
import frc.robot.subsystems.Health.HardwareHealth;

import static frc.robot.Constants.IntakeConstants.*;
import static frc.robot.Constants.BallVisionConstants.*;

public class IntakeSubsystem extends SubsystemBase{
    private final SparkFlex rollerMotor = new SparkFlex(IDConstants.kRollerPort, MotorType.kBrushless);
    private final SparkMax extendMotor = new SparkMax(IDConstants.kExtendPort, MotorType.kBrushless);

    private final SparkFlexConfig rollerConfig = new SparkFlexConfig();
    private final SparkMaxConfig extendConfig = new SparkMaxConfig();

    private final RelativeEncoder extendEncoder = extendMotor.getEncoder();
    private final SparkClosedLoopController extendPIDcontroller = extendMotor.getClosedLoopController();

    private boolean extendCurrentFault = false;
    private double extendOverCurrentStartSeconds = -1.0;
    private boolean rollerCollectingCommanded = false;
    private final IntakeBallCurrentDetector ballCurrentDetector =
        new IntakeBallCurrentDetector(
            kRollerBallDetectCurrentAmps,
            kRollerBallDetectStartupIgnoreSeconds,
            kRollerBallDetectDebounceSeconds);
    private final DoubleLogEntry rollerCurrentLog = new DoubleLogEntry(
        DataLogManager.getLog(), "/FRC8169/Intake/RollerCurrentAmps");
    private final BooleanLogEntry rollerCollectingCommandedLog = new BooleanLogEntry(
        DataLogManager.getLog(), "/FRC8169/Intake/RollerCollectingCommanded");
    private final BooleanLogEntry rollerBallDetectedLog = new BooleanLogEntry(
        DataLogManager.getLog(), "/FRC8169/Intake/RollerBallCurrentDetected");

    public IntakeSubsystem() {
        
        rollerConfig
            .inverted(true)
            .idleMode(IdleMode.kCoast)
            .smartCurrentLimit(70);

        extendConfig
            .inverted(false)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(60)
            .softLimit
            .forwardSoftLimitEnabled(true)
            .reverseSoftLimitEnabled(true)
            .forwardSoftLimit(kExtendFowardPosLimit)
            .reverseSoftLimit(kExtendReversePosLimit);
        extendConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(kP, kI, kD)
        .outputRange(kExtendMinOutput, kExtendMaxOutput);

        rollerMotor.configure(rollerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        extendMotor.configure(extendConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        extendEncoder.setPosition(extendEncoder.getPosition());

        HardwareHealth.getInstance().register(new CheckableSpark(rollerMotor, "Intake/Roller Motor"));
        HardwareHealth.getInstance().register(new CheckableSpark(extendMotor, "Intake/Extend Motor"));
       
    }

    public double getExtendPosition(){
        return extendEncoder.getPosition();
    }

    /**
     * Intake extension normalized to 0..1 using the same positions used by the
     * automatic Close/Extend states. This is only used for camera geometry.
     */
    public double getExtendFraction(){
        double retractedPosition = ExtendState.kClose.position;
        double extendedPosition = ExtendState.kExtend.position;

        if (Math.abs(extendedPosition - retractedPosition) < 1e-9) {
            return 0.0;
        }

        return MathUtil.clamp(
            (getExtendPosition() - retractedPosition) / (extendedPosition - retractedPosition),
            0.0,
            1.0);
    }

    /**
     * Live intake-Limelight XY offset from robot center. The camera moves forward
     * with the intake while remaining centered in robot Y.
     */
    public Translation2d getBallCameraRobotOffset(){
        double cameraX = kBallCameraRetractedXMeters
            + kBallCameraExtensionTravelMeters * getExtendFraction();

        return new Translation2d(cameraX, kBallCameraYMeters);
    }

    /**
     * Live intake-Limelight height above the floor. The intake linkage lowers the
     * camera from about 0.38 m when retracted to about 0.315 m when fully extended.
     */
    public double getBallCameraHeightMeters(){
        double fraction = getExtendFraction();
        return kBallCameraRetractedHeightMeters
            + (kBallCameraExtendedHeightMeters - kBallCameraRetractedHeightMeters) * fraction;
    }

    public void setRollerState(RollerAction action){
        rollerCollectingCommanded = action == RollerAction.kGetBall;
        rollerMotor.set(action.state);
    }

    public boolean hasDetectedBallFromRollerCurrent(){
        return ballCurrentDetector.isDetected();
    }

    public void clearRollerBallDetection(){
        ballCurrentDetector.clearDetection();
    }

    public void setExtendManual(ExtendManual speed){
        if (extendCurrentFault) {
            extendMotor.stopMotor();
            return;
        }
        extendMotor.set(speed.rate);
    }

    public void setExtendAuto(ExtendState state){
        if (extendCurrentFault) {
            extendMotor.stopMotor();
            return;
        }
        extendPIDcontroller.setSetpoint(state.position, ControlType.kPosition);
    }

    public void clearExtendCurrentFault(){
        extendMotor.stopMotor();
        extendCurrentFault = false;
        extendOverCurrentStartSeconds = -1.0;
    }

    public void stopextendMotor(){
        extendMotor.stopMotor();
    }

    public void stopRollerMotor(){
        rollerCollectingCommanded = false;
        rollerMotor.stopMotor();
    }


    @Override
    public void periodic(){
        double extendCurrentAmps = extendMotor.getOutputCurrent();
        double rollerCurrentAmps = rollerMotor.getOutputCurrent();
        ballCurrentDetector.update(
            Timer.getFPGATimestamp(),
            rollerCurrentAmps,
            rollerCollectingCommanded);

        if (!extendCurrentFault && extendCurrentAmps >= kExtendCurrentTripAmps) {
            if (extendOverCurrentStartSeconds < 0.0) {
                extendOverCurrentStartSeconds = Timer.getFPGATimestamp();
            } else if (Timer.getFPGATimestamp() - extendOverCurrentStartSeconds
                    >= kExtendCurrentTripSeconds) {
                extendCurrentFault = true;
                extendMotor.stopMotor();
                DriverStation.reportWarning(
                    "Intake extend motor stopped: over current "
                        + String.format("%.1f A", extendCurrentAmps),
                    false);
            }
        } else if (extendCurrentAmps < kExtendCurrentTripAmps) {
            extendOverCurrentStartSeconds = -1.0;
        }

        if (extendCurrentFault) {
            extendMotor.stopMotor();
        }

        SmartDashboard.putNumber("Intake / ExtendRelativePos", getExtendPosition());
        SmartDashboard.putNumber("Intake / ExtendCurrentAmps", extendCurrentAmps);
        SmartDashboard.putBoolean("Intake / ExtendCurrentFault", extendCurrentFault);
        SmartDashboard.putNumber("Intake / RollerCurrentAmps", rollerCurrentAmps);
        SmartDashboard.putNumber(
            "Intake / RollerBallCurrentThresholdAmps",
            kRollerBallDetectCurrentAmps);
        SmartDashboard.putBoolean(
            "Intake / RollerBallCurrentDetected",
            ballCurrentDetector.isDetected());
        SmartDashboard.putBoolean(
            "Intake / RollerBallCurrentDebouncing",
            ballCurrentDetector.isAboveThresholdDebouncing());
        rollerCurrentLog.append(rollerCurrentAmps);
        rollerCollectingCommandedLog.update(rollerCollectingCommanded);
        rollerBallDetectedLog.update(ballCurrentDetector.isDetected());
        SmartDashboard.putNumber("BallVision/IntakeExtensionFraction", getExtendFraction());
        SmartDashboard.putNumber("BallVision/CameraRobotX", getBallCameraRobotOffset().getX());
        SmartDashboard.putNumber("BallVision/CameraRobotY", getBallCameraRobotOffset().getY());
        SmartDashboard.putNumber("BallVision/CameraHeightMeters", getBallCameraHeightMeters());
    }
}
