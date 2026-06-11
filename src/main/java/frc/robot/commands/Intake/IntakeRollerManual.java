package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants.RollerAction;
import frc.robot.subsystems.Intake.IntakeSubsystem;

public class IntakeRollerManual extends Command{

    private IntakeSubsystem intakeSubsystem;
    private RollerAction rollerAction;

    public IntakeRollerManual(IntakeSubsystem intakeSubsystem, RollerAction rollerAction){
        this.intakeSubsystem = intakeSubsystem;
        this.rollerAction = rollerAction;
    }

  @Override
  public void initialize() {
    intakeSubsystem.setRollerState(rollerAction);
  }

  @Override
  public void execute() {
  }
  
  @Override
  public void end(boolean interrupted) {
    intakeSubsystem.setRollerState(RollerAction.kStop);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
