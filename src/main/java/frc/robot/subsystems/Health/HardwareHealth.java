package frc.robot.subsystems.Health;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.ArrayList;
import java.util.List;

public class HardwareHealth {
    private static HardwareHealth instance;
    private final List<Checkable> devices = new ArrayList<>();
    private boolean allHealthy = true;
    private boolean waringSig = false;

    public static HardwareHealth getInstance() {
        if (instance == null)
            instance = new HardwareHealth();
        return instance;
    }

    public void register(Checkable device) {
        devices.add(device);
    }

    public void periodicUpdate() {
        // 為了減少 CAN 負載與 Dashboard 更新頻率，建議每 1 秒 (50 幀) 跑一次
        allHealthy = true;
        for (Checkable device : devices) {
            SmartDashboard.putBoolean("Health/" + device.getName(), device.isHealthy());
            if (!device.isHealthy()) {
                allHealthy = false;
            }
            // // 這裡可以加上額外的警告機制，例如聲響或燈光提示
            // System.out.println("Warning: " + device.getName() + " is unhealthy!");
            // }
        }
    }

    public void periodic(){
        if(devices.size() > 0 && allHealthy){
            waringSig = true;
        }else{
            waringSig = !waringSig;
        }
        SmartDashboard.putBoolean("Health/All Healthy", waringSig);
    }
}
