package frc.robot.commands.Swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.Swerve.SwerveSubsytem;

public class SwerveAiming extends Command {

    private final SwerveSubsytem swerveSubsytem;
    private final ShooterSubsystem shooterSubsystem;
    private int shootMode;
    private static double xSpeed = 0;
    private static double ySpeed = 0;  

    private double currentRobotAngle = 0;
    private double targetAngle = 0;

    private static final PIDController pidController = new PIDController(
        DriveConstants.kPLockHeading,
        DriveConstants.kILockHeading,
        DriveConstants.kDLockHeading
    );
    

    public SwerveAiming(SwerveSubsytem swerveSubsytem,ShooterSubsystem shooterSubsystem, int shootMode, double xSpeed,double ySpeed) {
        this.swerveSubsytem = swerveSubsytem;
        this.shooterSubsystem = shooterSubsystem;
        this.shootMode = shootMode;
        this.xSpeed = xSpeed;
        this.ySpeed = ySpeed;

        pidController.enableContinuousInput(0, 360);
        pidController.setTolerance(DriveConstants.kAimingErrTolerence);
        pidController.setIZone(DriveConstants.kIzLockHeading);
        
        //addRequirements(swerveSubsytem);

        
    }

    public static boolean aimIsReady(){
        return pidController.getError() < DriveConstants.kAimingErrTolerence;
    }

    @Override
    public void initialize() {
        pidController.reset();
    }

    @Override
    public void execute() {
        double rotationSpeed = 0;

        currentRobotAngle = swerveSubsytem.getPose().getRotation().getDegrees() - 180;

        

        switch (shootMode) {
            case 0: // Shoot
                 // NEW: 用虛擬目標點取代真實 Hub 座標
                Translation2d virtualTarget = shooterSubsystem.getVirtualTargetLocation(swerveSubsytem);
                Translation2d toTarget = virtualTarget.minus(swerveSubsytem.getPose().getTranslation());

                Rotation2d targetFieldAngle = toTarget.getAngle();
                targetAngle = targetFieldAngle.getDegrees();

                double pidOutput = pidController.calculate(currentRobotAngle, targetAngle);

                // NEW: 角速度前饋 — 用切向速度分量除以距離
                double angularVelFF = 0.0;
                double distance = toTarget.getNorm();
                if (distance > 0.05) { // 避免除以太小的數字暴衝
                    ChassisSpeeds fieldVel = swerveSubsytem.getFieldRelativeSpeeds();
                    Translation2d vel = new Translation2d(fieldVel.vxMetersPerSecond, fieldVel.vyMetersPerSecond);

                    // 轉到「以瞄準方向為 X 軸」的座標系，Y 分量就是切向速度
                    Translation2d velInTargetFrame = vel.rotateBy(toTarget.getAngle().unaryMinus());
                    double tangentialVel = velInTargetFrame.getY();

                    angularVelFF = Math.toDegrees(tangentialVel / distance);
                }

                rotationSpeed = pidOutput + angularVelFF;
                swerveSubsytem.setChassisOutput(xSpeed*0.5, ySpeed*0.5, -rotationSpeed);

                SmartDashboard.putNumber("Shooter/SOTM AngularVelFF", angularVelFF);
                SmartDashboard.putNumber("Shooter/SOTM PidOutput", pidOutput);
                break;

            case 1: // Pass 
                targetAngle = 180.0;
                rotationSpeed = pidController.calculate(currentRobotAngle, targetAngle);
                swerveSubsytem.setChassisOutput(xSpeed*0.5, ySpeed*0.5, -rotationSpeed);
                break;

            default:
                break;
        }

        

        SmartDashboard.putNumber("targetAngle", targetAngle);
        SmartDashboard.putNumber("DeltaAngle", pidController.getError());
        SmartDashboard.putNumber("shoot Mode", shootMode);
        SmartDashboard.putBoolean("atSetpoint", pidController.atSetpoint());
        SmartDashboard.putBoolean("aimReady", aimIsReady());
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}