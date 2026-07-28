package frc.robot.commands.Swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ShooterConstant;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.Swerve.SwerveSubsytem;

public class SwerveAiming extends Command {

    private final SwerveSubsytem swerveSubsytem;
    private final ShooterSubsystem shooterSubsystem;
    private final int shootMode;

    private double currentRobotAngle = 0;
    private double targetAngle = 0;

    private final PIDController pidController = new PIDController(
        DriveConstants.kPLockHeading,
        DriveConstants.kILockHeading,
        DriveConstants.kDLockHeading
    );
    

    public SwerveAiming(SwerveSubsytem swerveSubsytem,ShooterSubsystem shooterSubsystem, int shootMode) {
        this.swerveSubsytem = swerveSubsytem;
        this.shooterSubsystem = shooterSubsystem;
        this.shootMode = shootMode;

        pidController.enableContinuousInput(0, 360);
        pidController.setTolerance(DriveConstants.kAimingErrTolerence);
        pidController.setIZone(DriveConstants.kIzLockHeading);
        
        addRequirements(swerveSubsytem);
    }

    @Override
    public void initialize() {
        pidController.reset();
    }

    @Override
    public void execute() {
        double rotationSpeed = 0;

        currentRobotAngle = swerveSubsytem.getPose().getRotation().getDegrees();

        switch (shootMode) {
            case 0: // Shoot
                Translation2d currentTarget = shooterSubsystem.getTargetHubLocation();
                Rotation2d targetFieldAngle = currentTarget.minus(swerveSubsytem.getPose().getTranslation()).getAngle();
                targetAngle = targetFieldAngle.getDegrees();

                rotationSpeed = pidController.calculate(currentRobotAngle, targetAngle);
                swerveSubsytem.setChassisOutput(0, 0, -rotationSpeed);
                System.out.println(rotationSpeed);
                
                break;

            case 1: // Pass 
                targetAngle = 180.0;
                rotationSpeed = pidController.calculate(currentRobotAngle, targetAngle);
                break;

            default:
                break;
        }

        

        SmartDashboard.putNumber("targetAngle", targetAngle);
        SmartDashboard.putNumber("DeltaAngle", pidController.getError());
        SmartDashboard.putNumber("shoot Mode", shootMode);
        SmartDashboard.putBoolean("atSetpoint", pidController.atSetpoint());
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}