package frc.robot.constants;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public abstract class HeadConstants {

    public static final int branchSensorId = 1;
    public static final int coralSensorId = 0;
    public static final int backSensorId = 2;
    public static final int frontSensorId = 3;
    public static final int leftId = 12;
    public static final int rightId = 11;

    public static final int currentLimit = 25;
    public static final IdleMode idleMode = IdleMode.kBrake;
    public static final int shortSensorSampleTime = 40; //milliseconds
    public static final int longSensorSampleTime = 80; //milliseconds
    public static final double coralActivationDist = 80; //millimeters
    public static final double branchLVL2ActivationDist = 700; //millimeters
    public static final double branchLVL3MinActivationDist = 375; //millimeters
    public static final double branchLVL3ActivationDist = 725; //millimeters
    public static final double branchLVL4ActivationDist = 400; //millimeters
    public static final double branchSigmaActivation = 5; //millimeters
    public static final boolean leftReversed = false;
    public static final boolean rightReversed = true;

    public static final double indexerVbus = 0.2;

    public static final double shootTime = 0.75; //seconds
    public static final double shootVBus = 1;
    public static final double shootSlowVBus = 0.33;
}
