// package frc.robot.commands.Swerve;

// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.Constants.DriveConstants;
// import frc.robot.subsystems.ShooterSubsystem;
// import frc.robot.subsystems.Swerve.SwerveSubsytem;

// public class SwerveAiming extends Command {
    
//     private final ShooterSubsystem shooterSubsystem;
//     private final SwerveSubsytem swerveSubsytem;
//     private int ShootMode = 0;

//     PIDController pidController = new PIDController(
//     DriveConstants.kPLockHeading, 
//     DriveConstants.kILockHeading, 
//     DriveConstants.kDLockHeading);

//     //private final SlewRateLimiter turningLimiter;

//     public SwerveAiming(ShooterSubsystem shooterSubsystem, SwerveSubsytem swerveSubsytem, int ShootMode) {
//         //this.turningLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);
//         this.shooterSubsystem = shooterSubsystem;
//         this.swerveSubsytem = swerveSubsytem;
//         this.ShootMode = ShootMode;

//         pidController.enableContinuousInput(-180, 180);

//         addRequirements(swerveSubsytem);
//     }


//     @Override
//     public void execute() {
//         switch (ShootMode) {
//             case 0: // shoot
//                 Translation2d currentTarget = shooterSubsystem.getTargetHubLocation();

//                 Rotation2d toHubAngle = currentTarget.minus(swerveSubsytem.getPose().getTranslation()).getAngle();
//                 double targetAngle = toHubAngle.getDegrees();
//                 double currentAngle = swerveSubsytem.getPose().getRotation().getDegrees();
//                 //double deltaAngle = MathUtil.inputModulus(targetAngle - currentAngle, -180, 180);

//                 double rotationSpeed = pidController.calculate(currentAngle, targetAngle);
//                 swerveSubsytem.setChassisOutput(0, 0, rotationSpeed);

//                 break;

//             case 1: // pass
//                 double passCurrentAngle =  swerveSubsytem.getRobotRotation().getDegrees();
//                 double passRotationSpeed = pidController.calculate(passCurrentAngle, 180);
//                 swerveSubsytem.setChassisOutput(0, 0, passRotationSpeed);

//                 break;

//             default:
//                 break;
//         }
//     }
// }
