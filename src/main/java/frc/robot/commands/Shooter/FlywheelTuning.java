package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class FlywheelTuning extends Command{
    private final ShooterSubsystem shooterSubsystem;

    public FlywheelTuning(ShooterSubsystem shooterSubsystem){
        this.shooterSubsystem = shooterSubsystem;
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        shooterSubsystem.FlywheelTuning();
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
