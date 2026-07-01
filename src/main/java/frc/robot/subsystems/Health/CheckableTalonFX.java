package frc.robot.subsystems.Health;

import com.ctre.phoenix6.hardware.TalonFX;

public class CheckableTalonFX implements Checkable {
    private final TalonFX telonFx;
    private final String name;

    public CheckableTalonFX(TalonFX telonFx, String name) {
        this.telonFx = telonFx;
        this.name = name;
    }

    @Override
    public boolean isHealthy() {
        // 檢查裝置是否在 CAN 網路上有回應 (檢查版本回傳的狀態)
        // if(!telonFx.getVersion().getStatus().isOK()) {
        //     System.out.println("TalonFX " + name + " error: " + telonFx.getVersion().getStatus().toString());
        // }
        return telonFx.getVersion().getStatus().isOK();
    }

    @Override
    public String getName() { return name; }
}