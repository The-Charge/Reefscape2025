package frc.robot.constants;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public abstract class HeadConstants {

    public static final int funnelSensorId = 1;
    public static final int shooterSensorId = 0;
    public static final int leftId = 12;
    public static final int rightId = 11;

    public static final int currentLimit = 25;
    public static final IdleMode idleMode = IdleMode.kBrake;
    public static final int sensorSampleTime = 40; //milliseconds
    public static final double sensorActivationDist = 80; //millimeters
    public static final double hasCoralDeactivationDelay = 0.5; //seconds
    public static final boolean leftReversed = false;
    public static final boolean rightReversed = true;

    public static final double indexerVbus = 0.2;

    public static final double shootTime = 0.75; //seconds
    public static final double shootVBus = 1;
}
