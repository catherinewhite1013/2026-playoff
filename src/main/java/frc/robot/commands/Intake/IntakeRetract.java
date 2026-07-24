package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.IntakeConstants.ExtendManual;
import frc.robot.Constants.IntakeConstants.ExtendState;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeRetract extends Command{
    
    private final IntakeSubsystem intakeSubsystem;

    public IntakeRetract(IntakeSubsystem intakeSubsystem){
        this.intakeSubsystem = intakeSubsystem;
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        if (intakeSubsystem.getExtendPosition() > IntakeConstants.kExtendReversePosLimit+20) {
            intakeSubsystem.setExtendManual(ExtendManual.kIn);
            intakeSubsystem.stopRollerMotor();
        } else{
            intakeSubsystem.setExtendManual(ExtendManual.kStop);
            intakeSubsystem.stopRollerMotor();
        }        

    }

    @Override
    public void end(boolean interrupted){
        intakeSubsystem.setExtendAuto(ExtendState.kExtend);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
    
}


