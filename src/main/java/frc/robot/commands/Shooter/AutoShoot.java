package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.StatusSubsystem;
import frc.robot.subsystems.Swerve.SwerveSubsytem;

public class AutoShoot extends Command{

    private final ShooterSubsystem shooterSubsystem;
    private final SwerveSubsytem swerveSubsytem;
    private final StatusSubsystem statusSubsystem;

    public AutoShoot(
        ShooterSubsystem shooterSubsystem,
        SwerveSubsytem swerveSubsytem,
        StatusSubsystem statusSubsystem){
        this.shooterSubsystem = shooterSubsystem;
        this.swerveSubsytem = swerveSubsytem;
        this.statusSubsystem = statusSubsystem;

        addRequirements(shooterSubsystem, statusSubsystem);
    }

    @Override
    public void initialize() {
      statusSubsystem.beginAutoShoot();
    }

    @Override
    public void execute() {
      shooterSubsystem.autoShoot(swerveSubsytem);
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
