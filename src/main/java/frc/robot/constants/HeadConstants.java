package frc.robot.constants;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public abstract class HeadConstants {

    public static final int funnelSensorId = 0;
    public static final int shooterSensorId = 1;
    public static final int leftId = 7;
    public static final int rightId = 8;

    public static final int currentLimit = 25;
    public static final IdleMode idleMode = IdleMode.kCoast;
    public static final int sensorSampleTime = 40; //milliseconds
    public static final double sensorActivationDist = 50; //millimeters
    public static final double hasCoralDeactivationDelay = 0.25; //seconds

    public static final double indexerVbus = 0.2;

    public static final double shootTime = 0.5;
    public static final double shootVBus = 0.5;
}
