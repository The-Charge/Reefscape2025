package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.LEDConstants;
import frc.robot.constants.TelemetryConstants;

public class LEDSubsystem extends SubsystemBase {

    public static abstract class Pattern {
        /**
         * Use for outputing to leds, prefer to use precalculated values for efficiency reasons
         */
        public abstract void evaluate(AddressableLEDBufferView segment);
    }
    public static abstract class Animation extends Pattern {
        /**
         * Use for updating output values, mostly meant for custom animations
         * MUST ONLY BE CALLED AT MOST ONCE PER UPDATE/FRAME
         */
        public abstract void update();
        /**
         * Use for resetting the animation back to the beginning
         */
        public abstract void reset();
    }
    
    private final AddressableLED leds;
    private final AddressableLEDBuffer buff;
    private final AddressableLEDBufferView fullBuffSeg;

    public LEDSubsystem() {
        leds = new AddressableLED(LEDConstants.ledPort);
        buff = new AddressableLEDBuffer(LEDConstants.ledLength);

        fullBuffSeg = buff.createView(0, buff.getLength() - 1);
        
        leds.setLength(LEDConstants.ledLength);
        leds.setColorOrder(LEDConstants.ledColorOrder);

        leds.start();
    }

    @Override
    public void periodic() {
        update();

        if(getCurrentCommand() == null)
            SmartDashboard.putString("LED RunningCommand", "None");
        else
            SmartDashboard.putString("LED RunningCommand", getCurrentCommand().getName());
    }

    public AddressableLEDBufferView fullBuff() {
        return fullBuffSeg;
    }

    private void update() {
        for(int i : LEDConstants.brokenIndicies) {
            if(i < 0 || i >= buff.getLength())
                continue;
            
            buff.setLED(i, Color.kBlack);
        }
        leds.setData(buff);
    }
}
