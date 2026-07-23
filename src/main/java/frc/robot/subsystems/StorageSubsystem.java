package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IDConstants;
import frc.robot.Constants.StorageConstant.ShooterFeedAction;
import frc.robot.Constants.StorageConstant.StorageAction;
import frc.robot.subsystems.Health.CheckableSpark;
import frc.robot.subsystems.Health.CheckableTalonFX;
import frc.robot.subsystems.Health.HardwareHealth;

public class StorageSubsystem extends SubsystemBase{
    private final TalonFX storageMotor = new TalonFX(IDConstants.kStoragePort);
    private final SparkFlex shooterFeedMotor = new SparkFlex(IDConstants.kShooterFeedPort, MotorType.kBrushless);
    private TalonFXConfiguration storageConfig = new TalonFXConfiguration();
    private SparkFlexConfig shooterFeedConfig = new SparkFlexConfig();

    public StorageSubsystem(){

        storageConfig.withMotorOutput(new MotorOutputConfigs()
            .withInverted(InvertedValue.CounterClockwise_Positive)
            .withNeutralMode(NeutralModeValue.Coast)
            .withPeakForwardDutyCycle(1)
            .withPeakReverseDutyCycle(-1));

        shooterFeedConfig
            .inverted(true)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(80);

        shooterFeedMotor.configure(shooterFeedConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        storageMotor.getConfigurator().apply(storageConfig);
        
        HardwareHealth.getInstance().register(new CheckableSpark(shooterFeedMotor, "Storage/shooter feed"));
        HardwareHealth.getInstance().register(new CheckableTalonFX(storageMotor, "Storage/conveyor"));
    }

    public void setStorageAction(StorageAction storageAction, ShooterFeedAction shooterFeedAction){
        storageMotor.set(storageAction.state);
        shooterFeedMotor.set(shooterFeedAction.state);
    }

    @Override
    public void periodic(){
    }
}
