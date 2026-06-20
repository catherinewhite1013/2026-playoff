package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Swerve.SwerveSubsytem;

public class SwerveAimingSubsystem extends SubsystemBase{
    
    public SwerveAimingSubsystem(){
    }

    public double turnAngleCal(ShooterSubsystem shooterSubsystem, SwerveSubsytem swerveSubsytem){
        Translation2d currentHubTarget = shooterSubsystem.getTargetHubLocation();

        Rotation2d toHubAngle = currentHubTarget.minus(swerveSubsytem.getPose().getTranslation()).getAngle();

        double targetAngle = toHubAngle.getDegrees();
        double currentAngle = swerveSubsytem.getPose().getRotation().getDegrees() % 360; //確保是 0-360度

        double deltaAngle =  targetAngle - currentAngle;

        if (deltaAngle > 180) {
            double oppositeDeltaAngle = 360-currentAngle+targetAngle;
            return oppositeDeltaAngle;
        } else  {
            return deltaAngle;
        }

        
        
    }

    @Override
    public void periodic() {}
}
