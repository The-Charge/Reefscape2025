package frc.robot.constants;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import swervelib.parser.PIDFConfig;

public abstract class ElevConstants {

    public static final int motorID = 19;
    public static final double maxVBus = 1;
    public static final double maxCurrent = 25;
    public static final PIDFConfig pidf = new PIDFConfig(0.3, 0, 0.01, 0);
    public static final double minPosTicks = 0;
    public static final double maxPosTicks = 101.224;
    public static final InvertedValue inverted = InvertedValue.Clockwise_Positive;
    public static final NeutralModeValue neutralMode = NeutralModeValue.Brake;
    public static final double tickToInConversion = 61.0 / (maxPosTicks - minPosTicks);
    public static final double targetThresholdInches = 0.2;
    public static final double targetThresholdSeconds = 0.1;

    //lowest funnel position is about 16.769 in from ground
    public static final double homeInches = 0;
    public static final double lvl1Inches = 18 - 16.769;
    public static final double lvl2Inches = 31.72 - 16.769;
    public static final double lvl3Inches = 47.59 - 16.769;
    public static final double lvl4Inches = 71.87 - 16.769;
    public static final double algaeLowInches = 0;
    public static final double algaeHighInches = 0;

    public static final String overrideInName = "Elev Target Override (In)";
    public static final String overrideTicksName = "Elev Target Override (Ticks)";
    public static final String overrideLVLName = "Elev Target Override (LVL)";
}
