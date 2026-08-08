// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OIConstants;
import frc.robot.Constants.IntakeConstants.ExtendManual;
import frc.robot.Constants.IntakeConstants.ExtendState;
import frc.robot.Constants.IntakeConstants.RollerAction;
import frc.robot.Constants.StorageConstant.StorageAction;
import frc.robot.commands.StorageCommand;
import frc.robot.commands.Intake.IntakeAuto;
import frc.robot.commands.Intake.IntakeExtendManual;
import frc.robot.commands.Intake.IntakeRetract;
import frc.robot.commands.Intake.IntakeRollerManual;
import frc.robot.commands.Shooter.FlywheelTuning;
import frc.robot.commands.Shooter.AutoPass;
import frc.robot.commands.Shooter.AutoShoot;
import frc.robot.commands.Swerve.PathfindToBallCluster;
import frc.robot.commands.Swerve.SwerveAiming;
import frc.robot.commands.Swerve.SwerveFieldRelative;
import frc.robot.commands.vision.ballFinding;
import frc.robot.logging.RobotTelemetry;
import frc.robot.match.HubShiftCalculator;
import frc.robot.match.HubShiftCalculator.AllianceColor;
import frc.robot.match.HubShiftCalculator.Input;
import frc.robot.match.HubShiftCalculator.MatchMode;
import frc.robot.match.HubShiftCalculator.Result;
import frc.robot.match.HubShiftCalculator.TestOverride;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.StatusSubsystem;
import frc.robot.subsystems.StorageSubsystem;
import frc.robot.subsystems.Swerve.SwerveSubsytem;

import java.util.Optional;
import java.util.Set;
import java.util.concurrent.atomic.AtomicReference;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
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
  //private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  private final StorageSubsystem storageSubsystem = new StorageSubsystem();
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  private final StatusSubsystem statusSubsystem = new StatusSubsystem();
  private final RobotTelemetry telemetry;
  

  private static CommandXboxController m_driverController = new CommandXboxController(
      OIConstants.kDriverControllerPort);
  private static CommandXboxController m_operatorController = new CommandXboxController(
      OIConstants.kOperatorControllerPort);


  // Create auto chooser
  private final SendableChooser<Command> autoChooser;
  private final SendableChooser<TestOverride> hubTestOverrideChooser = new SendableChooser<>();

  private final AtomicReference<Translation2d> lastScannedCluster = new AtomicReference<>();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the trigger bindings
    // SmartDashboard.putData(elevatorSubsystem);

    configureNamedCommands();
    autoChooser = AutoBuilder.buildAutoChooser(); // Default auto will be `Commands.none()`
    SmartDashboard.putData("Auto Mode", autoChooser);

    hubTestOverrideChooser.setDefaultOption("Disabled", TestOverride.DISABLED);
    hubTestOverrideChooser.addOption("Red won AUTO", TestOverride.RED);
    hubTestOverrideChooser.addOption("Blue won AUTO", TestOverride.BLUE);
    SmartDashboard.putData("GameTimer/Practice Override", hubTestOverrideChooser);

    telemetry = new RobotTelemetry(
        swerveSubsytem,
        m_driverController.getHID(),
        m_operatorController.getHID());
   
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
    
    m_driverController.rightTrigger().whileTrue(new IntakeAuto(intakeSubsystem, ExtendState.kExtend));
    m_driverController.leftTrigger().whileTrue(new IntakeAuto(intakeSubsystem, ExtendState.kClose));
    m_driverController.y().whileTrue(new IntakeRollerManual(intakeSubsystem, RollerAction.kStop));

    //operator
    m_operatorController.leftTrigger().whileTrue(  //aiming
      new ParallelCommandGroup(
        new SwerveAiming(swerveSubsytem,shooterSubsystem, 0),
        new AutoShoot(shooterSubsystem, swerveSubsytem, statusSubsystem,true)));
    m_operatorController.b().whileTrue(
      new ParallelCommandGroup(
        new SwerveAiming(swerveSubsytem, shooterSubsystem, 0),
        new AutoShoot(shooterSubsystem, swerveSubsytem, statusSubsystem, false)
      ));

    m_operatorController.leftBumper().whileTrue(new ParallelCommandGroup(
      new SwerveAiming(swerveSubsytem,shooterSubsystem, 1),
      new AutoPass(shooterSubsystem, swerveSubsytem)));

    m_operatorController.rightTrigger().whileTrue(new IntakeRetract(intakeSubsystem,storageSubsystem));
    m_operatorController.rightBumper().whileTrue(new StorageCommand(storageSubsystem, StorageAction.kIn));

    m_operatorController.pov(0).whileTrue(new IntakeExtendManual(intakeSubsystem, ExtendManual.kOut));  //intake
    m_operatorController.pov(180).whileTrue(new IntakeExtendManual(intakeSubsystem, ExtendManual.kIn));
    m_operatorController.a().whileTrue(new InstantCommand(()-> intakeSubsystem.stopextendMotor()));
    
    m_operatorController.x().whileTrue(new FlywheelTuning(shooterSubsystem));
    m_operatorController.y().whileTrue(new InstantCommand(()-> shooterSubsystem.calcShooterToHub(swerveSubsytem.getPose())));

  }

  private void setDefaultCommand() {
    swerveSubsytem.setDefaultCommand(new SwerveFieldRelative(swerveSubsytem,
        () -> m_driverController.getLeftY(), // X-Axis
        () -> m_driverController.getLeftX(), // Y-Axis
        () -> m_driverController.getRightX() // R-Axis
    ));
  }

  private void configureNamedCommands() {

    NamedCommands.registerCommand("AutoShootCommand",
      new ParallelCommandGroup(
        new SwerveAiming(swerveSubsytem,shooterSubsystem,0),
        new AutoShoot(shooterSubsystem, swerveSubsytem, statusSubsystem,true),
        new SequentialCommandGroup(
          new WaitUntilCommand(()-> shooterSubsystem.isReady() && SwerveAiming.aimIsReady()),
          new IntakeRetract(intakeSubsystem,storageSubsystem))));        
        

    NamedCommands.registerCommand("IntakeGetBall", 
      new IntakeAuto(intakeSubsystem, ExtendState.kExtend));

    
    NamedCommands.registerCommand("ScanAndGoToBallCluster",
        new SequentialCommandGroup(
            // 掃描階段：機器人在打完第一趟球後，短暫轉向掃描場地
            new ballFinding(
                swerveSubsytem,
                lastScannedCluster::set,
                0.5,   // 掃描 0.5 秒，依實際幀率調整
                false  // 若共用相機改成 true
            ),
            // 導航階段：往密度最高的地方開，開啟 intake 邊走邊撿
            new ParallelCommandGroup(
                new DeferredCommand(
                    () -> PathfindToBallCluster.build(
                        lastScannedCluster.get(),
                        getFallbackScanPose(), // 找不到球時的預設收球點
                        Rotation2d.fromDegrees(0)),
                    Set.of(swerveSubsytem)
                ),
                new IntakeAuto(intakeSubsystem, ExtendState.kExtend)
            )
        ));
}
private Pose2d getFallbackScanPose() {
    // 沒偵測到球時的保底位置，例如場地上固定的收球區中心
    return new Pose2d(6.0, 3.7, Rotation2d.fromDegrees(90));
}
    
    //NamedCommands.registerCommand("PassBall", getAutonomousCommand());


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return autoChooser.getSelected();
  }

  public void logTelemetry() {
    telemetry.log();
  }

  public void logAutonomousCommand(Command autonomousCommand) {
    telemetry.logAutonomousCommand(
        autonomousCommand == null ? "None" : autonomousCommand.getName());
  }

  public void updateMatchTimer() {
    TestOverride selectedOverride = Optional.ofNullable(hubTestOverrideChooser.getSelected())
        .orElse(TestOverride.DISABLED);
    Result result = HubShiftCalculator.calculate(new Input(
        DriverStation.getMatchTime(),
        getMatchMode(),
        DriverStation.getAlliance()
            .map(alliance -> alliance == Alliance.Red ? AllianceColor.RED : AllianceColor.BLUE)
            .orElse(AllianceColor.UNKNOWN),
        DriverStation.getGameSpecificMessage(),
        DriverStation.isFMSAttached(),
        selectedOverride));

    SmartDashboard.putNumber("GameTimer/Match Time", result.displayMatchTimeSeconds());
    SmartDashboard.putString("GameTimer/Shift Label", result.phase().label());
    SmartDashboard.putNumber("GameTimer/Shift CD", result.secondsToNextChange());
    // Deprecated typo retained temporarily for existing Elastic dashboard layouts.
    SmartDashboard.putBoolean("GameTimer/Hub Active", result.hubActive());
    SmartDashboard.putBoolean("GameTimer/Hub Status Known", result.hubStatusKnown());
    SmartDashboard.putString(
        "GameTimer/Hub Status",
        result.hubStatusKnown() ? (result.hubActive() ? "Active" : "Inactive") : "Unknown");
    SmartDashboard.putString("GameTimer/Data Source", result.dataSource().label());
    SmartDashboard.putString("GameTimer/Game Data", result.effectiveGameData());
    SmartDashboard.putString("GameTimer/Raw Game Data", result.rawGameData());
    SmartDashboard.putBoolean(
        "GameTimer/Test Override Enabled", result.testOverrideEnabled());
    SmartDashboard.putBoolean(
        "GameTimer/Test Override Applied", result.testOverrideApplied());
    SmartDashboard.putString("GameTimer/Test Override Selection", selectedOverride.displayName());
  }

  private static MatchMode getMatchMode() {
    if (DriverStation.isDisabled()) {
      return MatchMode.DISABLED;
    }
    if (DriverStation.isAutonomousEnabled()) {
      return MatchMode.AUTONOMOUS;
    }
    if (DriverStation.isTeleopEnabled()) {
      return MatchMode.TELEOP;
    }
    if (DriverStation.isTestEnabled()) {
      return MatchMode.TEST;
    }
    return MatchMode.UNKNOWN;
  }
}
