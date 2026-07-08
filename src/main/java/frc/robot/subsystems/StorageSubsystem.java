package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IDConstants;
import frc.robot.Constants.StorageConstant.ShooterFeedAction;
import frc.robot.Constants.StorageConstant.StorageAction;
import frc.robot.subsystems.Health.CheckableSpark;
import frc.robot.subsystems.Health.HardwareHealth;

public class StorageSubsystem extends SubsystemBase{
    private final SparkMax storageMotor = new SparkMax(IDConstants.kStoragePort, MotorType.kBrushless);
    private final SparkMax shooterFeedMotor = new SparkMax(IDConstants.kShooterFeedPort, MotorType.kBrushless);
    private SparkMaxConfig storageConfig = new SparkMaxConfig();
    private SparkMaxConfig shooterFeedConfig = new SparkMaxConfig();

    public StorageSubsystem(){

        storageConfig
        .inverted(false)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(IDConstants.kShooterFeedPort);

        shooterFeedConfig
        .inverted(false)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(80);

        HardwareHealth.getInstance().register(new CheckableSpark(shooterFeedMotor, "Storage/shooter feed"));
        HardwareHealth.getInstance().register(new CheckableSpark(storageMotor, "Storage/conveyor"));
    }

    public void setStorageAction(StorageAction storageAction, ShooterFeedAction shooterFeedAction){
        storageMotor.set(storageAction.state);
        shooterFeedMotor.set(shooterFeedAction.state);
    }

    @Override
    public void periodic(){
    }
}
