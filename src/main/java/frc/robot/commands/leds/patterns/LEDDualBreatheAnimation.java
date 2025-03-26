package frc.robot.commands.leds.patterns;

import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.subsystems.LEDSubsystem.Animation;

public class LEDDualBreatheAnimation extends Animation {
    
    private final Color col1, col2;
    private final double timeOn, timeOff;
    private final double transitionPerc;
    private final Timer time;
    private final Timer colTime;

    private double modTime, lastModTime;
    private Color outCol;

    public LEDDualBreatheAnimation(Color col1, Color col2, double timeOn, double timeOff, double transitionPercentage) {
        this.col1 = col1;
        this.col2 = col2;
        this.timeOn = timeOn;
        this.timeOff = timeOff;
        transitionPerc = transitionPercentage;

        time = new Timer();
        time.start();

        colTime = new Timer();
        colTime.start();

        lastModTime = 0;
    }

    @Override
    public void update() {
        modTime = time.get() % (timeOn * 2 + timeOff * 2);

        if(modTime < lastModTime || lastModTime < timeOn + timeOff && timeOn + timeOff <= modTime)
            colTime.reset();

        Color col = Color.kBlack;
        if(modTime < timeOn)
            col = col1;
        else if(timeOn + timeOff <= modTime && modTime < timeOn * 2 + timeOff)
            col = col2;
        
        double strength;
        double perc = colTime.get() / timeOn;
        if(perc < transitionPerc)
            strength = Math.sin((Math.PI * perc) / (2 * transitionPerc));
        else if(1 - transitionPerc <= perc)
            strength = Math.cos((Math.PI * (perc - 1 + transitionPerc)) / (2 * transitionPerc));
        else
            strength = 1;
        
        outCol = new Color(
            col.red * strength,
            col.green * strength,
            col.blue * strength
        );

        lastModTime = modTime;
    }
    @Override
    public void reset() {
        time.reset();
        colTime.reset();
    }
    @Override
    public void evaluate(AddressableLEDBufferView seg) {
        for(int i = 0; i < seg.getLength(); i++) {
            seg.setLED(i, outCol);
        }
    }
}
