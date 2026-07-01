package frc.robot.subsystems.Health;

import com.studica.frc.AHRS;

public class CheckableNavX implements Checkable {
    private final AHRS navx;
    private final String name;

    public CheckableNavX(AHRS navx, String name) {
        this.navx = navx;
        this.name = name;
    }

    @Override
    public boolean isHealthy() {
        // 必須已連線且完成初始化校準
        return navx.isConnected() && !navx.isCalibrating();
    }

    @Override
    public String getName() { return name; }
}