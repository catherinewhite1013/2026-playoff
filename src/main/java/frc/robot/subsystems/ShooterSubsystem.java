package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.IDConstants;
import frc.robot.commands.Swerve.SwerveAiming;
import frc.robot.subsystems.Health.CheckableSpark;
import frc.robot.subsystems.Health.CheckableTalonFX;
import frc.robot.subsystems.Health.HardwareHealth;
import frc.robot.subsystems.Swerve.SwerveSubsytem;

import static frc.robot.Constants.ShooterConstant.*;

public class ShooterSubsystem extends SubsystemBase{
    
    //secondary flywheel
    private final SparkMax secondaryFlywheelMotor = new SparkMax(IDConstants.kShooterSecondaryPort, MotorType.kBrushless);
    private final SparkMaxConfig secondaryFlywheelConfig = new SparkMaxConfig();
    private final RelativeEncoder secEncoder = secondaryFlywheelMotor.getEncoder();
    private final SparkClosedLoopController secFlywheelClosedLoopCtrl = secondaryFlywheelMotor.getClosedLoopController();

    //main flywheel
    private final TalonFX mainFlywheelMotor = new TalonFX(IDConstants.kShooterMainPort);
    private final VelocityVoltage FlywheelVVControl = new VelocityVoltage(0);
    TalonFXConfiguration mainFlywheelConfiguration = new TalonFXConfiguration();

    //Tree map
    private final InterpolatingDoubleTreeMap mainFlywheelSpeed = new InterpolatingDoubleTreeMap();
    private final InterpolatingDoubleTreeMap secFlywheelSpeed = new InterpolatingDoubleTreeMap();

    private double targetMainRPM = 0;
    private double targetSecRPM = 0;

    public ShooterSubsystem() {
        
        //Main flywheel
        mainFlywheelConfiguration.withMotorOutput(new MotorOutputConfigs()
            .withInverted(InvertedValue.CounterClockwise_Positive)
            .withNeutralMode(NeutralModeValue.Coast)
            .withPeakForwardDutyCycle(1)
            .withPeakReverseDutyCycle(0));

        mainFlywheelConfiguration.withCurrentLimits(new CurrentLimitsConfigs()
            .withSupplyCurrentLimit(80)
            .withSupplyCurrentLimitEnable(true)
            .withStatorCurrentLimit(320)
            .withStatorCurrentLimitEnable(true));

        mainFlywheelConfiguration.withFeedback(new FeedbackConfigs()
            .withSensorToMechanismRatio(kMainFlywheelBeltRatio));

        mainFlywheelConfiguration.withSlot0(new Slot0Configs()
            .withKV(kMainFlywheelkV)
            .withKS(kMainFlywheelkS)
            .withKP(kMainFlywheelkP)
            .withKI(0)
            .withKD(0));

        mainFlywheelMotor.getConfigurator().apply(mainFlywheelConfiguration);

        FlywheelVVControl.withSlot(0);

        //Secondary flywheel
        secondaryFlywheelConfig
            .inverted(false)
            .idleMode(IdleMode.kCoast)
            .smartCurrentLimit(70);
        
        secondaryFlywheelConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(kSecondaryFlywheelKP, kSecondaryFlywheelKI, kSecondaryFlywheelKD)
            .outputRange(0, 1)
            .feedForward
                .kV(kSecondaryFlywheelKV)
                .kS(kSecondaryFlywheelKS);

        secondaryFlywheelMotor.configure(secondaryFlywheelConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        setupInterpolationTable();

        HardwareHealth.getInstance().register(new CheckableSpark(secondaryFlywheelMotor, "Shooter/sec Flywheel"));
        HardwareHealth.getInstance().register(new CheckableTalonFX(mainFlywheelMotor, "Shooter/main flywheel"));

        SmartDashboard.putNumber("Tuning/Main Flywheel RPM", 0);
        SmartDashboard.putNumber("Tuning/Sec Flywheel RPM", 0);
    }

    private void setupInterpolationTable(){
        for (double[] data : kShooterDataMap){
            mainFlywheelSpeed.put(data[0], data[1]);
            secFlywheelSpeed.put(data[0], data[2]);
        }
    }

    //Main flywheel
    public double getMainFlywheelSpeed(){
        return mainFlywheelMotor.getVelocity(true).getValueAsDouble()*60;
    }

    public void setMainFlywheelRPM(double targetMainRPM){
        mainFlywheelMotor.setControl(FlywheelVVControl.withVelocity(targetMainRPM/60)); //RPS
    }

    //Secondary flywheel
    public double getSecFlywheelSpeed(){
        return secEncoder.getVelocity();
    }

    public void setSecFlywheelRPM(double targetSecRPM){
        secFlywheelClosedLoopCtrl.setSetpoint(targetSecRPM, ControlType.kVelocity);
    }

    //Aim & shoot
    public void autoAim(double distance){
        targetMainRPM = mainFlywheelSpeed.get(distance);    //調表格資料
        targetSecRPM = secFlywheelSpeed.get(distance);

        applySetSpeed();    //轉
    }

    public void autoShoot(SwerveSubsytem swerveSubsytem){
        Translation2d ShooterFieldPosition = swerveSubsytem.getPose().getTranslation()
            .plus(kRobotToShooter.rotateBy(swerveSubsytem.getPose().getRotation()));    //Shooter 場地位置

        Translation2d currentHubPosition = getTargetHubLocation();

        double distanceToHub = ShooterFieldPosition.getDistance(currentHubPosition);
        this.autoAim(distanceToHub);
    }

    public void applySetSpeed(){
        secFlywheelClosedLoopCtrl.setSetpoint(targetSecRPM, ControlType.kVelocity);

        double motorTargetRPS = targetMainRPM/60;
        mainFlywheelMotor.setControl(FlywheelVVControl.withVelocity(motorTargetRPS));
    }

    public void manualShoot(){
        this.targetMainRPM = 4000.0;
        this.targetSecRPM = 1000.0;

        this.applySetSpeed();
    }

    public void stopAll(){
        mainFlywheelMotor.stopMotor();
        secondaryFlywheelMotor.stopMotor();
        targetMainRPM = 0;
        targetSecRPM = 0;
    }

    public boolean isReady(){
        if (targetMainRPM == 0 && targetSecRPM == 0) {
            return false;
        }

        boolean mainSpeedReady = Math.abs(getMainFlywheelSpeed() - targetMainRPM) < kMainFlywheelErrTolerence;
        boolean secSpeedReady = Math.abs(getSecFlywheelSpeed() - targetSecRPM) < kSecondaryFlywheelErrTolerence;

        return mainSpeedReady && secSpeedReady;
    }

    public double getChargeProgress(){
        if (targetMainRPM <= 0 || targetSecRPM <= 0) {
            return 0.0;
        }

        double mainProgress = Math.abs(getMainFlywheelSpeed()) / targetMainRPM;
        double secProgress = Math.abs(getSecFlywheelSpeed()) / targetSecRPM;
        return MathUtil.clamp(Math.min(mainProgress, secProgress), 0.0, 1.0);
    }

    //ball passing
    public void AutoPass(SwerveSubsytem swerveSubsytem){
        var allience = DriverStation.getAlliance();
        
        if (allience.isPresent() && allience.get() == DriverStation.Alliance.Red) {
            double distanceToRedPass = swerveSubsytem.getPose().getTranslation().getX() - kRedPassLocation;
            this.autoAim(distanceToRedPass);
        } else {
            double distanceToBluePass = swerveSubsytem.getPose().getTranslation().getX() - kBluePassLocation;
            this.autoAim(distanceToBluePass);
        }
    }

    //get distance
    public Translation2d getTargetHubLocation(){
        var allience = DriverStation.getAlliance();
        
        if (allience.isPresent() && allience.get() == DriverStation.Alliance.Red) {
            return kRedHubLocation;
        } else {
            return kBlueHubLocation;
        }
    }

    //Tuning
    public void calcShooterToHub(Pose2d robotPose) {    //for tuning
        Translation2d targetLocation = getTargetHubLocation();

        Transform2d robotToShooter = new Transform2d(kRobotToShooter, new Rotation2d());
        Pose2d shooterPose = robotPose.transformBy(robotToShooter);

        Translation2d targetVector = targetLocation.minus(shooterPose.getTranslation());

        double distanceToHub = targetVector.getNorm();

        

        SmartDashboard.putNumber("Shooter/Calc Dist to Hub", distanceToHub);
        SmartDashboard.putNumber("Shooter/Calc TargetRPM", mainFlywheelSpeed.get(distanceToHub));
        SmartDashboard.putNumber("Shooter/Calc TargetAngle", secFlywheelSpeed.get(distanceToHub));
    }

    public void FlywheelTuning(){
        this.targetMainRPM = SmartDashboard.getNumber("Tuning/Main Flywheel RPM", 0);
        this.targetSecRPM = SmartDashboard.getNumber("Tuning/Sec Flywheel RPM", 0);

        this.applySetSpeed();
    }

    @Override
    public void periodic() {

        // double targetMainRPM = mainFlywheelMotor.getClosedLoopReference().getValueAsDouble()*60;
        // double targetSecRPM = secFlywheelClosedLoopCtrl.getMAXMotionSetpointVelocity();

        SmartDashboard.putNumber("Shooter/Main Actual RPM", getMainFlywheelSpeed());
        SmartDashboard.putNumber("Shooter/MAin Target RPM", targetMainRPM);
        SmartDashboard.putNumber("Shooter/Sec Actual RPM", getSecFlywheelSpeed());
        SmartDashboard.putNumber("Shooter/Sec Target RPM", targetSecRPM);
        SmartDashboard.putBoolean("Shooter/isReady", isReady());

        

        //SmartDashboard.putBoolean("Shooter/IsReady", isReady());
    }

}
