// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Swerve;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve.SwerveSubsytem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SwerveRobotRelative extends Command {
  private final SwerveSubsytem swerveSubsystem;
  private final Supplier<Integer> povSpdFunction;
  private final SlewRateLimiter limiter;

  /** Creates a new SwerveRobotRelative. */
  public SwerveRobotRelative(SwerveSubsytem swerveSubsytem, Supplier<Integer> povSpdFunction) {
    this.swerveSubsystem = swerveSubsytem;
    this.povSpdFunction = povSpdFunction;
    // Use addRequirements() here to declare subsystem dependencies.

    addRequirements(swerveSubsytem);

    this.limiter = new SlewRateLimiter(2);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    limiter.reset(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Apply slew rate to joystick input to make robot input smoother and mulitply
    // by max speed
    double speed = limiter.calculate(1);
    double rad = Math.toRadians(povSpdFunction.get());
    swerveSubsystem.setChassisOutput(speed * Math.cos(rad), -speed * Math.sin(rad), 0, false, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerveSubsystem.stopModules();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
