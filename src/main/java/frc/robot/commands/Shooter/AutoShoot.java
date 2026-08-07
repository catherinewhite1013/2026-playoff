package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.StatusSubsystem;
import frc.robot.subsystems.Swerve.SwerveSubsytem;

public class AutoShoot extends Command{

    private final ShooterSubsystem shooterSubsystem;
    private final SwerveSubsytem swerveSubsytem;
    private final StatusSubsystem statusSubsystem;
    private boolean intakeAlive = true;

    public AutoShoot(
      ShooterSubsystem shooterSubsystem,
      SwerveSubsytem swerveSubsytem,
      StatusSubsystem statusSubsystem,
      boolean intakeAlive){
        this.shooterSubsystem = shooterSubsystem;
        this.swerveSubsytem = swerveSubsytem;
        this.statusSubsystem = statusSubsystem;
        this.intakeAlive = intakeAlive;

        addRequirements(shooterSubsystem, statusSubsystem);
    }

    @Override
    public void initialize() {
      statusSubsystem.beginAutoShoot();
    }

    @Override
    public void execute() {
      shooterSubsystem.autoShoot(swerveSubsytem, intakeAlive);
      statusSubsystem.updateAutoShoot(
          shooterSubsystem.isReady(),
          shooterSubsystem.getChargeProgress());
    }

    @Override
    public void end(boolean interrupted) {
      shooterSubsystem.stopAll();
      statusSubsystem.endAutoShoot();
    }

    @Override
    public boolean isFinished() {
      return false;
    }
}
