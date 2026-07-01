package frc.robot.subsystems.Health;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.Faults;

public class CheckableSpark implements Checkable {
    private final SparkBase spark;
    private final String name;

    public CheckableSpark(SparkBase spark, String name) {
        this.spark = spark;
        this.name = name;
    }

    @Override
    public boolean isHealthy() {
        // kOk 代表目前通訊正常且無硬體錯誤
        // return spark.getLastError() == com.revrobotics.REVLibError.kOk;
        // if(spark.getLastError() != com.revrobotics.REVLibError.kOk) {
        // System.out.println("Spark " + name + " error: " + spark.getLastError());
        // }
        boolean noError = spark.getLastError() == com.revrobotics.REVLibError.kOk;
        Faults f = spark.getFaults();
        boolean hasSensor = !f.sensor;
        return noError && hasSensor;
    }

    @Override
    public String getName() {
        return name;
    }
}