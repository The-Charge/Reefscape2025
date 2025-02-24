package frc.robot.commands.leds.patterns;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.constants.LEDConstants;
import frc.robot.subsystems.LEDSubsystem.Animation;

public class LEDStepsAnimation extends Animation {
    
    private final Color col1, col2;
    private final int scale;
    private final double ledsPerSec;
    private final Timer time;


    public LEDStepsAnimation(Color col1, Color col2, int scale, LinearVelocity scrollSpeed) {
        this.col1 = col1;
        this.col2 = col2;
        this.scale = scale;
        
        ledsPerSec = scrollSpeed.in(Units.MetersPerSecond) / LEDConstants.ledSpacing.in(Units.Meters);

        time = new Timer();
        time.start();
    }

    @Override
    public void update() {}
    @Override
    public void reset() {
        time.reset();
    }
    @Override
    public void evaluate(AddressableLEDBufferView seg) {
        final double scaledSegLength = seg.getLength() / (double) scale;

        for(int i = 0; i < seg.getLength(); i++) {
            boolean isCol1 = mod((i - time.get() * ledsPerSec), scaledSegLength) < 0.5 * scaledSegLength;

            seg.setLED(i, isCol1 ? col1 : col2);
        }
    }

    private double mod(double n, double d) {
        //https://en.wikipedia.org/wiki/Modulo#:~:text=3%5D%20promotes-,floored%20division,-%2C%20for%20which%20the
        return n - d * Math.floor(n / d);
    }
}
