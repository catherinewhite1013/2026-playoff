package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class ManualShoot extends Command{
    private ShooterSubsystem shooterSubsystem;

    public ManualShoot(ShooterSubsystem shooterSubsystem){
        this.shooterSubsystem = shooterSubsystem;
    }

    @Override
    public void initialize(){
        shooterSubsystem.manualShoot();
    }

    @Override
    public void execute() {
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
