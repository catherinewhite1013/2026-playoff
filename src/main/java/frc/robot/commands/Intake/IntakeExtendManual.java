package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants.ExtendManual;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeExtendManual extends Command{
    private IntakeSubsystem intakeSubsystem;
    private ExtendManual extendManual;

    public IntakeExtendManual(IntakeSubsystem intakeSubsystem, ExtendManual extendManual){
        this.intakeSubsystem = intakeSubsystem;
        this.extendManual = extendManual;
    }

  @Override
  public void initialize() {
    intakeSubsystem.clearExtendCurrentFault();
    intakeSubsystem.setExtendManual(extendManual);
  }

  @Override
  public void execute() {
  }

  @Override
  public void end(boolean interrupted) {
    intakeSubsystem.setExtendManual(ExtendManual.kStop);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
