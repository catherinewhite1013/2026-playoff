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

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IDConstants;
import frc.robot.Constants.IntakeConstants.ExtendManual;
import frc.robot.Constants.IntakeConstants.ExtendState;
import frc.robot.Constants.IntakeConstants.RollerAction;
import frc.robot.subsystems.Health.CheckableSpark;
import frc.robot.subsystems.Health.HardwareHealth;

import static frc.robot.Constants.IntakeConstants.*;

public class IntakeSubsystem extends SubsystemBase{
    private final SparkFlex rollerMotor = new SparkFlex(IDConstants.kRollerPort, MotorType.kBrushless);
    private final SparkMax extendMotor = new SparkMax(IDConstants.kExtendPort, MotorType.kBrushless);

    private final SparkFlexConfig rollerConfig = new SparkFlexConfig();
    private final SparkMaxConfig extendConfig = new SparkMaxConfig();

    private final RelativeEncoder extendEncoder = extendMotor.getEncoder();
    private final SparkClosedLoopController extendPIDcontroller = extendMotor.getClosedLoopController();

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

    public void setRollerState(RollerAction action){
        rollerMotor.set(action.state);
    }

    public void setExtendManual(ExtendManual speed){
        extendMotor.set(speed.rate);
    }

    public void setExtendAuto(ExtendState state){
        extendPIDcontroller.setSetpoint(state.position, ControlType.kPosition);
    }

    public void stopextendMotor(){
        extendMotor.stopMotor();
    }

    public void stopRollerMotor(){
        rollerMotor.stopMotor();
    }


    @Override
    public void periodic(){
        SmartDashboard.putNumber("Intake / ExtendRelativePos", getExtendPosition());
    }
}
