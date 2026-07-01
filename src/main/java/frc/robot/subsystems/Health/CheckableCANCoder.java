package frc.robot.subsystems.Health;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.MagnetHealthValue;

public class CheckableCANCoder implements Checkable {
    private final CANcoder cancoder;
    private final String name;

    public CheckableCANCoder(CANcoder cancoder, String name) {
        this.cancoder = cancoder;
        this.name = name;
    }

    @Override
    public boolean isHealthy() {
        // 1. 檢查連線狀態與基本硬體錯誤 (是否在 CAN 網路上)
        boolean isConnected = cancoder.getStickyFault_Hardware().getValue() == false;
        
        // 2. 檢查磁鐵狀態 (磁鐵太遠或遺失會回傳 Bad)
        // MagnetHealthValue: 0 (Invalid), 1 (Good), 2 (Orange/Weak), 3 (Red/Bad)
        MagnetHealthValue magnetHealth = cancoder.getMagnetHealth().getValue();
        boolean isMagnetOk = (magnetHealth == MagnetHealthValue.Magnet_Green);

        // 同時滿足「連線正常」且「磁鐵位置正確」才視為正常
        return isConnected && isMagnetOk;
    }

    @Override
    public String getName() {
        return name;
    }
}
