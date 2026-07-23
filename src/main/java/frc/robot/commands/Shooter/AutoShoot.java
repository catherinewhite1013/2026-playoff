package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.Swerve.SwerveSubsytem;

public class AutoShoot extends Command{

    private final ShooterSubsystem shooterSubsystem;
    private final SwerveSubsytem swerveSubsytem;

    public AutoShoot(ShooterSubsystem shooterSubsystem, SwerveSubsytem swerveSubsytem){
        this.shooterSubsystem = shooterSubsystem;
        this.swerveSubsytem = swerveSubsytem;

        addRequirements(shooterSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
      shooterSubsystem.autoShoot(swerveSubsytem);
    }

    @Override
    public void end(boolean interrupted) {
      shooterSubsystem.stopAll();
    }

    @Override
    public boolean isFinished() {
      return false;
    }
}
