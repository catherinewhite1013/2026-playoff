package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.StorageConstant.ShooterFeedAction;
import frc.robot.Constants.StorageConstant.StorageAction;
import frc.robot.subsystems.StorageSubsystem;

public class StorageCommand extends InstantCommand{
    private StorageSubsystem storageSubsystem;
    private StorageAction storageAction;

    public StorageCommand(StorageSubsystem storageSubsystem){
        this.storageSubsystem = storageSubsystem;
        this.storageAction = storageAction;
    }

    @Override
    public void initialize(){
        storageSubsystem.setStorageAction(StorageAction.kIn, ShooterFeedAction.kIn);
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted){
        storageSubsystem.setStorageAction(StorageAction.kStop, ShooterFeedAction.kStop);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}