// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Swerve;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;

public class SwerveModule extends SubsystemBase {
  private SparkMax driveMotor;
  private SparkMax turningMotor;

  private SparkMaxConfig driveConfig;
  private SparkMaxConfig turningConfig;

  private RelativeEncoder driveEncoder;
  private RelativeEncoder turningEncoder;

  private SparkClosedLoopController builtinTurningPidController;

  private CANcoder absoluteEncoder;

  private boolean DriveMotorReversed;
  private boolean TurningMotorReversed;

  // private ResetMode kResetMode = ResetMode.kResetSafeParameters;

  private String moduleName;

  // Special UI variables for swerve simulation
  private MechanismLigament2d simTurnCmd;
  private MechanismLigament2d simDirectionCmd;

  private MechanismLigament2d simTurnReal;
  private MechanismLigament2d simDirectionReal;

  /** Creates a new SwerveModule. */
  public SwerveModule(
      int driveMotorId,
      int turningMotorId,
      int absoluteEncoderId,
      boolean DriveMotorReversed,
      boolean TurningMotorReversed,
      String name) {
    moduleName = name;

    // Create absolute encoder
    absoluteEncoder = new CANcoder(absoluteEncoderId);

    driveMotor = new SparkMax(driveMotorId, MotorType.kBrushless);
    turningMotor = new SparkMax(turningMotorId, MotorType.kBrushless);

    driveEncoder = driveMotor.getEncoder();
    turningEncoder = turningMotor.getEncoder();

    driveConfig = new SparkMaxConfig();
    turningConfig = new SparkMaxConfig();

    builtinTurningPidController = turningMotor.getClosedLoopController();

    



    driveConfig
        .smartCurrentLimit(60)
        .idleMode(IdleMode.kBrake)
        .inverted(DriveMotorReversed)
        .encoder.positionConversionFactor(ModuleConstants.kDriveEncoderRot2Meter)
                .velocityConversionFactor(ModuleConstants.kDriveEncoderRPM2MeterPerSec);

    turningConfig
        .smartCurrentLimit(40)
        .idleMode(IdleMode.kBrake)
        .inverted(TurningMotorReversed)
        .encoder.positionConversionFactor(ModuleConstants.kTurningEncoderRot2Rad)
                .velocityConversionFactor(ModuleConstants.kTurningEncoderRPM2RadPerSec);
        

    turningConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .positionWrappingEnabled(true)
        .positionWrappingMaxInput(Math.PI)
        .positionWrappingMinInput(-Math.PI)
        .pid(ModuleConstants.kPTurning, ModuleConstants.kITurning, ModuleConstants.kDTurning)
        .iZone(0.0)
        .outputRange(-1, 1);

        driveMotor.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        turningMotor.configure(turningConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    

    resetEncoders();

    // Thanks to Alec for this code!
    // >-----------S-I-M------------<//

    // Create the mechanism 2d canvas and get the root
    Mechanism2d mod = new Mechanism2d(6, 6);
    MechanismRoot2d root = mod.getRoot("root", 3, 3);

    // // Add simTurn to the root, add direction to turn, then add it to smart
    // // dashboard
    simTurnCmd = root.append(new MechanismLigament2d("Swerve Turn", 2, 1.75));
    simDirectionCmd = simTurnCmd
        .append(new MechanismLigament2d("Wheel Speed", 1, 0, 6, new Color8Bit(Color.kPurple)));
    SmartDashboard.putData(moduleName + " commanded Turn", mod);

    // // ------------//

    // // Do the same thing but for the real module state
    Mechanism2d mod2 = new Mechanism2d(6, 6);
    MechanismRoot2d root2 = mod2.getRoot("root", 3, 3);

    simTurnReal = root2.append(new MechanismLigament2d("Swerve Turn", 2, 1.75));
    simDirectionReal = simTurnReal
        .append(new MechanismLigament2d("Wheel Speed", 1, 0, 6, new Color8Bit(Color.kPurple)));
    SmartDashboard.putData(moduleName + "  real Turn", mod2);

    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber(moduleName + "  speed", getDriveVelocity());
  }

  public double getDrivePosition() {
    return driveEncoder.getPosition();
  }

  public double getTurningPosition() {
    return turningEncoder.getPosition();
  }

  public double getDriveVelocity() {
    return driveEncoder.getVelocity();
  } 

  public double getTurningVelocity() {
    return turningEncoder.getVelocity();
  }

  public SwerveModulePosition getPosition() {
    return (new SwerveModulePosition(
        getDrivePosition(), new Rotation2d(getTurningPosition())));
  }

  /*
   * Convert absolute value of the encoder to radians and then subtract the radian
   * offset
   * then check if the encoder is reversed.
   */
  public double getAbsoluteEncoderRad() {
    return absoluteEncoder.getAbsolutePosition().getValueAsDouble() * 2 * Math.PI;
  }

  // Set turning encoder to match absolute encoder value with gear offsets applied
  public void resetEncoders() {
    driveEncoder.setPosition(0);
    turningEncoder.setPosition(getAbsoluteEncoderRad());
  }

  // Get swerve module current state, aka velocity and wheel rotation
  public SwerveModuleState getState() {
    return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
  }

  @SuppressWarnings("deprecation")
  public void setDesiredState(SwerveModuleState state) {
    // Check if new command has high driving power
    if (Math.abs(state.speedMetersPerSecond) < 0.001) {
      stop();
      return;
    }
  // Optimize swerve module state to do fastest rotation movement, aka never
    // rotate more than 90*
  state = SwerveModuleState.optimize(state, getState().angle);

    // Scale velocity down using robot max speed
    driveMotor.set(
        state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond * DriveConstants.kMotorMaxOutput);

    // Use PID to calculate angle setpoint
    builtinTurningPidController.setSetpoint(state.angle.getRadians(), ControlType.kPosition, ClosedLoopSlot.kSlot0);
    

    simTurnCmd.setAngle(state.angle); // .plus(Rotation2d.fromDegrees(90))
    simDirectionCmd.setAngle(state.speedMetersPerSecond > 0 ? 0 : 180);
    simDirectionCmd.setLength(Math.abs(state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond));

    simTurnReal.setAngle(getAbsoluteEncoderRad() * 360); // +90
    simDirectionReal
        .setAngle(getDriveVelocity() > 0 ? 0 : 180);
    simDirectionReal.setLength(Math.abs(getDriveVelocity() / DriveConstants.kPhysicalMaxSpeedMetersPerSecond));

    SmartDashboard.putString("Swerve[" + moduleName + "] state", state.toString());
  }

  // Stop all motors on module
  public void stop() {
    driveMotor.set(0);
    turningMotor.set(0);
  }

   // Motor and SparkMax methods for Monitor
  public double[] getMotorsCurrent() {
    return (new double[] { driveMotor.getOutputCurrent(), turningMotor.getOutputCurrent() });
  }
  
  public double[] getMotorsTemp() {
    return (new double[] { driveMotor.getMotorTemperature(), turningMotor.getMotorTemperature() });
  }
  
  public void setSmartCurrentLimiter(int driveLimit, int turningLimit) {
    driveConfig.smartCurrentLimit(driveLimit);
    turningConfig.smartCurrentLimit(turningLimit);
  }
}
