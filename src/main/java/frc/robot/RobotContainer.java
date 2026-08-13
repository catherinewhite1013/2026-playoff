// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OIConstants;
import frc.robot.Constants.IntakeConstants.ExtendManual;
import frc.robot.Constants.IntakeConstants.ExtendState;
import frc.robot.Constants.IntakeConstants.RollerAction;
import frc.robot.Constants.StorageConstant.StorageAction;
import static frc.robot.Constants.BallVisionConstants.*;
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
import frc.robot.commands.vision.BallVisionTracker;
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
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
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

    // TEST: hold B to slowly rotate-search with the intake-side Limelight.
    // Intake extends/runs immediately while SEARCH is active. After FUEL is seen,
    // the SAME button press transitions to pathfinding at <= 1.0 m/s.
    // Release B: stop ONLY the roller; keep the intake extended.
    m_driverController.b().whileTrue(buildContinuousBallCollectionCommand());

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
    m_operatorController.b().whileTrue(new InstantCommand(()-> intakeSubsystem.stopextendMotor()));
    
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
    
    NamedCommands.registerCommand("IntakeStop", 
      new IntakeRollerManual(intakeSubsystem, RollerAction.kStop));

    
    NamedCommands.registerCommand(
        "ScanAndGoToBallCluster",
        buildBallClusterAutoCommand(true));

    // PathPlanner Auto command: continuously SEARCH -> CHASE -> SEARCH for a
    // bounded amount of time, then return control to the next Auto node.
    // Use this as a Named Command node in the PathPlanner Auto command tree.
    NamedCommands.registerCommand(
        "CollectBallsAuto",
        new DeferredCommand(
            () -> buildContinuousBallCollectionCommand()
                .withTimeout(kBallAutoCollectTimeoutSeconds)
                .finallyDo(interrupted -> {
                    // End the auto collection segment safely. Keep the intake extended,
                    // matching Driver-B behavior, but stop roller and drivetrain output.
                    intakeSubsystem.stopRollerMotor();
                    swerveSubsytem.stopModules();
                    SmartDashboard.putString("BallVision/SearchState", "AUTO_DONE");
                }),
            Set.of(swerveSubsytem, intakeSubsystem)
        )
    );
}

private Command buildBallSearchIntakeCommand() {
    return new StartEndCommand(
        () -> {
            intakeSubsystem.setExtendAuto(ExtendState.kExtend);
            intakeSubsystem.setRollerState(RollerAction.kGetBall);
        },
        () -> {
            // Driver released B (or command was interrupted): stop collecting, but
            // intentionally DO NOT command ExtendState.kClose. The closed-loop extend
            // setpoint remains at kExtend, so the intake stays out.
            intakeSubsystem.stopRollerMotor();
        },
        intakeSubsystem
    );
}

private Command buildBallClusterAutoCommand(boolean useFallbackWhenNoBall) {
    // Legacy/one-shot autonomous behavior: one search, one path, then finish.
    // Kept for the existing ScanAndGoToBallCluster NamedCommand.
    Command oneShotSearchThenChase = new SequentialCommandGroup(
        new InstantCommand(() -> lastScannedCluster.set(null)),
        new ballFinding(
            swerveSubsytem,
            intakeSubsystem,
            lastScannedCluster::set,
            0.5,
            false
        ),
        new DeferredCommand(
            () -> PathfindToBallCluster.build(
                lastScannedCluster.get(),
                useFallbackWhenNoBall ? getFallbackScanPose() : null,
                swerveSubsytem.getPose()),
            Set.of(swerveSubsytem)
        )
    );

    return new ParallelCommandGroup(
        oneShotSearchThenChase,
        new IntakeAuto(intakeSubsystem, ExtendState.kExtend)
    );
}

/**
 * Continuous FUEL collection used by Driver B and the timed PathPlanner
 * CollectBallsAuto NamedCommand. This command intentionally never finishes by
 * itself; the Driver button release or the autonomous timeout ends it.
 */
private Command buildContinuousBallCollectionCommand() {
    BallVisionTracker tracker = new BallVisionTracker(swerveSubsytem, intakeSubsystem);

    Command continuousSearchAndChase = Commands.repeatingSequence(
        new InstantCommand(() -> lastScannedCluster.set(null)),

        // SEARCH: rotate until a valid cluster is found, then compare briefly.
        new ballFinding(
            swerveSubsytem,
            intakeSubsystem,
            lastScannedCluster::set,
            0.5,
            false
        ),

        // CHASE: pathfind to the selected cluster. If it disappears while still
        // far away, cancel the stale path and immediately start SEARCH again.
        new DeferredCommand(
            () -> buildTrackedBallChase(tracker, lastScannedCluster.get()),
            Set.of(swerveSubsytem)
        )
    );

    return new ParallelCommandGroup(
        continuousSearchAndChase,
        tracker,
        buildBallSearchIntakeCommand()
    );
}

private Command buildTrackedBallChase(
        BallVisionTracker tracker,
        Translation2d selectedTarget) {
    if (selectedTarget == null) {
        SmartDashboard.putString("BallVision/SearchState", "REACQUIRE");
        return Commands.none();
    }

    AtomicBoolean lostTarget = new AtomicBoolean(false);
    double[] chaseStartTimestamp = new double[] {-1.0};

    Command pathCommand = PathfindToBallCluster.buildContinuous(
        () -> tracker.getTrackedTargetOr(selectedTarget),
        swerveSubsytem::getPose,
        swerveSubsytem
    ).beforeStarting(() -> {
        lostTarget.set(false);
        chaseStartTimestamp[0] = Timer.getFPGATimestamp();
        tracker.beginTrackingTarget(selectedTarget);
        SmartDashboard.putString("BallVision/SearchState", "CHASE");
        SmartDashboard.putNumber("BallVision/ChaseTargetX", selectedTarget.getX());
        SmartDashboard.putNumber("BallVision/ChaseTargetY", selectedTarget.getY());
    });

    Command lostTargetCommand = new WaitUntilCommand(() -> {
        // Give PathPlanner and the observer time to take ownership after SEARCH.
        // The target was positively validated by ballFinding immediately before this.
        if (chaseStartTimestamp[0] < 0.0 || Timer.getFPGATimestamp() - chaseStartTimestamp[0] < kBallChaseStartupGraceSeconds) {
            return false;
        }

        Translation2d trackedTarget = tracker.getTrackedTargetOr(selectedTarget);
        double robotDistanceToTrackedTarget = swerveSubsytem.getPose().getTranslation().getDistance(trackedTarget);

        // Near the intake, losing the target is expected because the ball moves under
        // the camera. Finish the last short approach instead of spinning away.
        if (robotDistanceToTrackedTarget <= kBallFinishApproachDistanceMeters) {
            return false;
        }

        boolean lost = !tracker.isSelectedTargetStillVisible(selectedTarget);
        if (lost) {
            lostTarget.set(true);
        }
        return lost;
    });

    Command collectedCommand = new WaitUntilCommand(() -> {
        Translation2d trackedTarget = tracker.getTrackedTargetOr(selectedTarget);
        double distance = swerveSubsytem.getPose().getTranslation().getDistance(trackedTarget);
        SmartDashboard.putNumber("BallVision/TrackedTargetDistanceMeters", distance);
        return distance <= kBallCollectionDistanceMeters;
    }).andThen(Commands.waitSeconds(kBallCollectSettleSeconds));

    return pathCommand.raceWith(lostTargetCommand, collectedCommand).finallyDo(interrupted -> {
            if (lostTarget.get()) {
                SmartDashboard.putString("BallVision/SearchState", "LOST_REACQUIRE");
            } else {
                SmartDashboard.putString("BallVision/SearchState", "NEXT_SEARCH");
            }
        });
}

private Pose2d getFallbackScanPose() {
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