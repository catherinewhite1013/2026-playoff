package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants.AngleManual;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeAngleManual extends Command{
    private IntakeSubsystem intakeSubsystem;
    private AngleManual angleManual;

    public IntakeAngleManual(IntakeSubsystem intakeSubsystem, AngleManual angleManual){
        this.intakeSubsystem = intakeSubsystem;
        this.angleManual = angleManual;
    }

  @Override
  public void initialize() {
    intakeSubsystem.setAngleManual(angleManual);
  }

  @Override
  public void execute() {
  }

  @Override
  public void end(boolean interrupted) {
    intakeSubsystem.setAngleManual(AngleManual.kStop);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
