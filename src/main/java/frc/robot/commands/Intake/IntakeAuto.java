package frc.robot.commands.Intake;

import static frc.robot.Constants.IntakeConstants.kRollerStartMinAngle;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.IntakeConstants.AngleState;
import frc.robot.Constants.IntakeConstants.RollerAction;
import frc.robot.subsystems.Intake.IntakeSubsystem;

public class IntakeAuto extends InstantCommand{
    private IntakeSubsystem intakeSubsystem;
    private AngleState angleState;

    public IntakeAuto(IntakeSubsystem intakeSubsystem, AngleState angleState){
        this.intakeSubsystem = intakeSubsystem;
        this.angleState = angleState;
    }

    @Override
    public void initialize(){
        switch (angleState) {
            case kExtend:
                intakeSubsystem.setAngleAuto(AngleState.kExtend);
                if (intakeSubsystem.getAngleAbsPosition() >= kRollerStartMinAngle) {
                    intakeSubsystem.setRollerState(RollerAction.kGetBall);
                } else{
                    intakeSubsystem.setRollerState(RollerAction.kStop);
                }
                
                break;
        
            default:
                intakeSubsystem.setAngleAuto(AngleState.kClose);
                intakeSubsystem.setRollerState(RollerAction.kStop);
                break;
        }
    }
}
