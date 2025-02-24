package frc.robot.constants;

import java.util.Map;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.AddressableLED.ColorOrder;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;

public abstract class LEDConstants {
    
    public static final int ledPort = 0;
    public static final int ledLength = 113;
    public static final ColorOrder ledColorOrder = ColorOrder.kGRB;
    public static final Distance ledSpacing = Units.Meters.of(1 / 60.0); //60 leds per meter

    public static final Color chargeGreen = new Color("#008800");
    public static final Color chargeGold = new Color("#ffaa00");
}
