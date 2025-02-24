package frc.robot.commands.leds.patterns;

import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.subsystems.LEDSubsystem.Animation;

public class LEDScanAnimation extends Animation {

    private final Color col;
    private final double cycleTime;
    private final Timer time;
    
    private int pulseWidth;

    public LEDScanAnimation(Color color, int pulseLength, double cycleTime) {
        col = color;
        pulseWidth = pulseLength;
        this.cycleTime = cycleTime;

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
        final double offsetRange = seg.getLength() - pulseWidth;
        double offset = (offsetRange * 0.5) * (1 - Math.cos((2 * Math.PI * time.get()) / cycleTime));

        for(int i = 0; i < seg.getLength(); i++) {
            double strength = 0;
            if(offset < i && i < offset + pulseWidth) {
                strength = Math.sin((Math.PI / pulseWidth) * (i - offset));
            }

            Color pixel = new Color(
                col.red * strength,
                col.green * strength,
                col.blue * strength
            );

            seg.setLED(i, pixel);
        }
    }
}
