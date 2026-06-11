package frc.robot.subsystems.Intake;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkAbsoluteEncoder;
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

import static frc.robot.Constants.IntakeConstants.*;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IDConstants;
import frc.robot.Constants.IntakeConstants.AngleState;

public class IntakeSubsystem extends SubsystemBase{
    private final SparkFlex rollerMotor = new SparkFlex(IDConstants.kRollerPort, MotorType.kBrushless);
    private final SparkMax angleMotor = new SparkMax(IDConstants.kAnglePort, MotorType.kBrushless);

    private final SparkFlexConfig rollerConfig = new SparkFlexConfig();
    private final SparkMaxConfig angleConfig = new SparkMaxConfig();

    private final RelativeEncoder angleEncoder = angleMotor.getEncoder();
    private final SparkAbsoluteEncoder angleAbsoluteEncoder = angleMotor.getAbsoluteEncoder();
    private final SparkClosedLoopController anglePIDcontroller = angleMotor.getClosedLoopController();

    public IntakeSubsystem() {
        
        rollerConfig
            .inverted(false)
            .idleMode(IdleMode.kCoast)
            .smartCurrentLimit(70);

        angleConfig
            .inverted(false)
            .idleMode(IdleMode.kCoast)
            .smartCurrentLimit(60)
            .softLimit
            .forwardSoftLimitEnabled(true)
            .reverseSoftLimitEnabled(true)
            .forwardSoftLimit(kAngleFowardPosLimit)
            .reverseSoftLimit(kAngleReversePosLimit);
        angleConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
        .pid(kP, kI, kD)
        .outputRange(kAngleMinOutput, kAngleMaxOutput);

        rollerMotor.configure(rollerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        angleMotor.configure(angleConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        angleEncoder.setPosition(angleAbsoluteEncoder.getPosition());
       
    }

    public double getAnglePosition(){
        return angleEncoder.getPosition();
    }

    public double getAngleAbsPosition(){
        return angleAbsoluteEncoder.getPosition();
    }

    public void setRollerState(RollerAction action){
        rollerMotor.set(action.state);
    }

    public void setAngleManual(AngleManual speed){
        angleMotor.set(speed.rate);
    }

    public void setAngleAuto(AngleState state){
        anglePIDcontroller.setSetpoint(state.position, ControlType.kPosition);
    }

    public void stopAngleMotor(){
        angleMotor.stopMotor();
    }


    @Override
    public void periodic(){
        SmartDashboard.putNumber("Intake / AngleAbsPos", getAngleAbsPosition());
        SmartDashboard.putNumber("Intake / AngleRelativePos", getAnglePosition());
    }
}
