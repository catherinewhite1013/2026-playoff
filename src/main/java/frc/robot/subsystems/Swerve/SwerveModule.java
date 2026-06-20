package frc.robot.subsystems.Swerve;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public interface SwerveModule {
    /** 獲取目前模組的狀態（速度與角度） */
    SwerveModuleState getState();

    /** 獲取目前模組的位置（里程計使用） */
    SwerveModulePosition getPosition();

    /** 設定目標狀態 */
    void setDesiredState(SwerveModuleState state);

    /** 停止所有電機 */
    void stop();

    /** 重設編碼器 */
    void resetEncoders();

    /** 獲取驅動電機位置 (公尺) */
    double getDrivePosition();

    /** 獲取轉向電機角度 (弧度) */
    double getTurningPosition();

    /** 獲取驅動速度 (公尺/秒) */
    double getDriveVelocity();

    /** 獲取轉向速度 (弧度/秒) */
    double getTurningVelocity();

    /** 獲取絕對編碼器弧度 */
    double getAbsoluteEncoderRad();

    // 選配：監控相關
    double[] getMotorsCurrent();

    double[] getMotorsTemp();

    void printInfo();
}