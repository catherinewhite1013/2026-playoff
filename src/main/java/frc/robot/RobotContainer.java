// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OIConstants;
import frc.robot.Constants.IntakeConstants.ExtendManual;
import frc.robot.Constants.IntakeConstants.ExtendState;
import frc.robot.Constants.StorageConstant.StorageAction;
import frc.robot.commands.StorageCommand;
import frc.robot.commands.Intake.IntakeAuto;
import frc.robot.commands.Intake.IntakeExtendManual;
import frc.robot.commands.Shooter.AutoShoot;
import frc.robot.commands.Swerve.SwerveAiming;
import frc.robot.commands.Swerve.SwerveFieldRelative;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.StorageSubsystem;
import frc.robot.subsystems.Swerve.SwerveSubsytem;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsytem swerveSubsytem = new SwerveSubsytem();
  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  private final StorageSubsystem storageSubsystem = new StorageSubsystem();
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  

  private static CommandXboxController m_driverController = new CommandXboxController(
      OIConstants.kDriverControllerPort);
  private static CommandXboxController m_operatorController = new CommandXboxController(
      OIConstants.kOperatorControllerPort);


  // Create auto chooser
  private final SendableChooser<Command> autoChooser;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the trigger bindings
    // SmartDashboard.putData(elevatorSubsystem);

    configureNamedCommands();
    autoChooser = AutoBuilder.buildAutoChooser(); // Default auto will be `Commands.none()`
    SmartDashboard.putData("Auto Mode", autoChooser);

   
    // stateSubsystem.setLEDState(StatusSubsystem.LEDState.RAINBOW);

    configureBindings();
    setDefaultCommand();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  

  // Define Commands for Controller binding, NamedCommand

  private void configureBindings() {
    //drive
    m_driverController.start().whileTrue(new InstantCommand(() -> swerveSubsytem.zeroHeading()));

    //operator
    m_operatorController.leftBumper().whileTrue(  //aiming
      new ParallelCommandGroup(
        new SwerveAiming(shooterSubsystem, swerveSubsytem, 0),
        new AutoShoot(shooterSubsystem, swerveSubsytem)));

    m_operatorController.rightBumper().whileTrue(new StorageCommand(storageSubsystem, StorageAction.kIn));  //shooter ball convey

    m_operatorController.pov(0).whileTrue(new IntakeExtendManual(intakeSubsystem, ExtendManual.kOut));  //intake
    m_operatorController.pov(180).whileTrue(new IntakeExtendManual(intakeSubsystem, ExtendManual.kIn));
    m_operatorController.y().whileTrue(new IntakeAuto(intakeSubsystem, ExtendState.kExtend));
    m_operatorController.a().whileTrue(new IntakeAuto(intakeSubsystem, ExtendState.kClose));


  }

  private void setDefaultCommand() {
    swerveSubsytem.setDefaultCommand(new SwerveFieldRelative(swerveSubsytem,
        () -> -m_driverController.getLeftY(), // X-Axis
        () -> -m_driverController.getLeftX(), // Y-Axis
        () -> -m_driverController.getRightX() // R-Axis
    ));
  }

  private void configureNamedCommands() {

    NamedCommands.registerCommand("AutoShootCommand",
      new ParallelCommandGroup(
        new SwerveAiming(shooterSubsystem, swerveSubsytem, 0),
        new AutoShoot(shooterSubsystem, swerveSubsytem)));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return autoChooser.getSelected();
  }
}