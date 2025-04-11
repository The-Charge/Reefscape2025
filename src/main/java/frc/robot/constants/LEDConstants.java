package frc.robot.constants;

import java.util.List;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.AddressableLED.ColorOrder;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.subsystems.ElevSubsystem;

public abstract class LEDConstants {
    
    public static final int ledPort = 9;
    public static final int ledLength = 142;
    public static final ColorOrder ledColorOrder = ColorOrder.kGRB;
    public static final Distance ledSpacing = Units.Meters.of(1 / 60.0); //60 leds per meter
    public static final int[] brokenIndicies = new int[] {
        ledLength - 12 * 3 + 1
    };

    public static final Color chargeGreen = new Color("#008800");
    public static final Color chargeGold = new Color("#ffaa00");
    public static final Color orange = new Color("#ff2200");
    public static final Color white = new Color("#ffffff");

    public static final double endgameSecs = 30; //seconds left for endgame
    public static final double endgameDisplayTime = 30; //amount of time the endgame pattern is displayed for
    public static final double endgameRumbleTime = 2; //amount of time the controllers rumble for

    public static final List<ElevSubsystem.Level> branchAlignLevels = List.of(ElevSubsystem.Level.LVL2, ElevSubsystem.Level.LVL3, ElevSubsystem.Level.LVL4);
}
