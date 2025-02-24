package frc.robot.commands.leds;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.leds.patterns.LEDBreatheAnimation;
import frc.robot.constants.LEDConstants;
import frc.robot.subsystems.LEDSubsystem;

public class LEDManager extends Command {
    
    private final LEDSubsystem leds;
    
    // private LEDScanAnimation scan;
    // private LEDPattern rainbow;
    // private LEDStepsAnimation steps;
    private LEDBreatheAnimation breathe;

    public LEDManager(LEDSubsystem ledSub) {
        leds = ledSub;
        addRequirements(leds);

        // scan = new LEDScanAnimation(LEDConstants.chargeGreen, 16, 6);
        // rainbow = LEDPattern.rainbow(255, 255)
        //     .scrollAtAbsoluteSpeed(Units.MetersPerSecond.of(1), LEDConstants.ledSpacing);
        // steps = new LEDStepsAnimation(LEDConstants.chargeGreen, LEDConstants.chargeGold, 4, Units.MetersPerSecond.of(1));
        breathe = new LEDBreatheAnimation(LEDConstants.chargeGreen, LEDConstants.chargeGold, 5, 0.2, 0.4);
    }

    @Override
    public void execute() {
        // scan.update();
        // scan.evaluate(leds.fullBuff());

        // rainbow.applyTo(leds.fullBuff());

        // steps.update();
        // steps.evaluate(leds.fullBuff());

        breathe.update();
        breathe.evaluate(leds.fullBuff());
    }

    @Override
    public boolean isFinished() {
        return false;
    }
    @Override
    public boolean runsWhenDisabled() {
        return true;
    }
}
