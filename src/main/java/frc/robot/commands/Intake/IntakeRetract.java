package frc.robot.commands.Intake;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstant;
import frc.robot.Constants.IntakeConstants.ExtendManual;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeRetract extends Command{
    
    private final IntakeSubsystem intakeSubsystem;

    public IntakeRetract(IntakeSubsystem intakeSubsystem){
        this.intakeSubsystem = intakeSubsystem;
    }

    @Override
    public void initialize() {
        if (intakeSubsystem.getExtendPosition() > IntakeConstants.kExtendReversePosLimit+5) {
            intakeSubsystem.setExtendManual(ExtendManual.kIn);
            intakeSubsystem.stopRollerMotor();
        } else{
            intakeSubsystem.setExtendManual(ExtendManual.kStop);
            intakeSubsystem.stopRollerMotor();
        }
    }

    @Override
    public void execute() {
        

    }

    @Override
    public void end(boolean interrupted){
        intakeSubsystem.setExtendManual(ExtendManual.kStop);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
    
}


