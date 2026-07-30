package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.IntakeConstants.ExtendState;
import frc.robot.Constants.IntakeConstants.RollerAction;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeAuto extends InstantCommand{
    private IntakeSubsystem intakeSubsystem;
    private ExtendState extendState;

    public IntakeAuto(IntakeSubsystem intakeSubsystem, ExtendState extendState){
        this.intakeSubsystem = intakeSubsystem;
        this.extendState = extendState;
    }

    @Override
    public void initialize(){
        intakeSubsystem.clearExtendCurrentFault();
        switch (extendState) {
            case kExtend:
                intakeSubsystem.setExtendAuto(ExtendState.kExtend);
                intakeSubsystem.setRollerState(RollerAction.kGetBall);
                break;
            
            case kClose:
                intakeSubsystem.setExtendAuto(ExtendState.kClose);
                intakeSubsystem.setRollerState(RollerAction.kStop);
                break;
                
            default:
                break;
        }
    }
}
