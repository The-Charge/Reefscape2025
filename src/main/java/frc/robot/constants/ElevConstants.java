package frc.robot.constants;

import swervelib.parser.PIDFConfig;

public abstract class ElevConstants {

    public static final int motorID = 5;
    public static final double maxVBus = 1;
    public static final double maxCurrent = 20;
    public static final PIDFConfig pidf = new PIDFConfig(0.05, 0.002, 0, 0);
    public static final double minPosTicks = 0;
    public static final double maxPosTicks = Integer.MAX_VALUE;
    public static final double tickToInConversion = 1;
    public static final double targetThresholdInches = 0.3;
    public static final double targetThresholdSeconds = 0.1;

    //lowest funnel position is about 16.769 in from ground
    public static final double homeInches = 0;
    public static final double lvl1Inches = 18 - 16.769;
    public static final double lvl2Inches = 31.72 - 16.769;
    public static final double lvl3Inches = 47.59 - 16.769;
    public static final double lvl4Inches = 71.87 - 16.769;

    public static final String overrideInName = "Elev Target Override (In)";
    public static final String overrideTicksName = "Elev Target Override (Ticks)";
    public static final String overrideLVLName = "Elev Target Override (LVL)";
}
