package frc.robot.constants;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import swervelib.parser.PIDFConfig;

public abstract class AlgaeManipConstants {
    
    public static final int pivotID = 6;
    public static final int pivotCurrentLimit = 25;
    public static final IdleMode pivotIdle = IdleMode.kBrake;
    public static final boolean pivotInverted = true;
    
    public static final PIDFConfig pivotPID = new PIDFConfig(0, 0, 0, 0);
    public static final double pivotMinTicks = 0;
    public static final double pivotMaxTicks = Double.MAX_VALUE;
    public static final double pivotTicksToDegConversion = 1;

    public static final int rollerID = 7;
    public static final int rollerCurrentLimit = 25;
    public static final IdleMode rollerIdle = IdleMode.kCoast;
    public static final boolean rollerInverted = false;

    public static final double targetThresholdDeg = 0.4;
    public static final double targetThresholdSeconds = 0.1;

    public static final double spinVbus = 0.5;
    public static final double restingPos = 0;
    public static final double activePos = 100;
}
