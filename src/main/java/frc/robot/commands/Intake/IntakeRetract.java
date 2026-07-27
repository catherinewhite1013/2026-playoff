package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.IntakeConstants.ExtendManual;
import frc.robot.Constants.IntakeConstants.ExtendState;
import frc.robot.Constants.IntakeConstants.RollerAction;
import frc.robot.Constants.StorageConstant.ShooterFeedAction;
import frc.robot.Constants.StorageConstant.StorageAction;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.StorageSubsystem;

public class IntakeRetract extends Command{
    
    private final IntakeSubsystem intakeSubsystem;
    private final StorageSubsystem storageSubsystem;

    public IntakeRetract(IntakeSubsystem intakeSubsystem, StorageSubsystem storageSubsystem){
        this.intakeSubsystem = intakeSubsystem;
        this.storageSubsystem = storageSubsystem;
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        storageSubsystem.setStorageAction(StorageAction.kIn, ShooterFeedAction.kIn);
        if (intakeSubsystem.getExtendPosition() > IntakeConstants.kExtendReversePosLimit + 23) {
            intakeSubsystem.setExtendManual(ExtendManual.kIn);
            intakeSubsystem.setRollerState(RollerAction.kGetBall);
        } else{
            intakeSubsystem.setExtendManual(ExtendManual.kStop);
            intakeSubsystem.stopRollerMotor();
        }        

    }

    @Override
    public void end(boolean interrupted){
        storageSubsystem.setStorageAction(StorageAction.kStop, ShooterFeedAction.kStop);
        intakeSubsystem.setExtendAuto(ExtendState.kExtend);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
    
}


