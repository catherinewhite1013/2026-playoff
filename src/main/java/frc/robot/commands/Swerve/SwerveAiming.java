package frc.robot.commands.Swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ShooterConstant;
//import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.Swerve.SwerveSubsytem;

public class SwerveAiming extends Command {
    
    //private final ShooterSubsystem shooterSubsystem;
    private final SwerveSubsytem swerveSubsytem;
    private int ShootMode = 0;

    private double targetAngle = 0;

    PIDController pidController = new PIDController(
    DriveConstants.kPLockHeading, 
    DriveConstants.kILockHeading, 
    DriveConstants.kDLockHeading);

    //private final SlewRateLimiter turningLimiter;

    public SwerveAiming(/*ShooterSubsystem shooterSubsystem,*/ SwerveSubsytem swerveSubsytem, int ShootMode) {
        //this.turningLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);
        //this.shooterSubsystem = shooterSubsystem;
        this.swerveSubsytem = swerveSubsytem;
        this.ShootMode = ShootMode;

        pidController.enableContinuousInput(-180, 180);

        addRequirements(swerveSubsytem);
    }

    @Override
    public void initialize() {
        pidController.reset();
    }

    @Override
    public void execute() {
        
        switch (ShootMode) {
            case 0: // shoot
                Translation2d currentTarget = ShooterConstant.kBlueHubLocation;//shooterSubsystem.getTargetHubLocation();

                Rotation2d targetFieldAngle = currentTarget.minus(swerveSubsytem.getPose().getTranslation()).getAngle(); //change to shooter position later
                Rotation2d targetRobotRelativeAngle = targetFieldAngle.minus(swerveSubsytem.getPose().getRotation());
                targetAngle = targetRobotRelativeAngle.getDegrees();

                double rotationSpeed = pidController.calculate(0, targetAngle);
                swerveSubsytem.setChassisOutput(0, 0, rotationSpeed);

                
                break;

            case 1: // pass
                double passCurrentAngle =  swerveSubsytem.getRobotRotation().getDegrees();
                double passRotationSpeed = pidController.calculate(passCurrentAngle, 180);
                swerveSubsytem.setChassisOutput(0, 0, passRotationSpeed);

                break;

            default:
                break;
        }
    SmartDashboard.putNumber("targetAngle", targetAngle);
    SmartDashboard.putNumber("shoot Mode", ShootMode);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
    
}