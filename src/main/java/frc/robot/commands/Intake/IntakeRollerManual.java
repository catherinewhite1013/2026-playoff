package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.IntakeConstants.RollerAction;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeRollerManual extends InstantCommand{

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
}
