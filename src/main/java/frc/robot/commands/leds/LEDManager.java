package frc.robot.commands.leds;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.leds.patterns.LEDDualBreatheAnimation;
import frc.robot.commands.leds.patterns.LEDScanAnimation;
import frc.robot.commands.leds.patterns.LEDStepsAnimation;
import frc.robot.constants.LEDConstants;
import frc.robot.subsystems.ElevSubsystem;
import frc.robot.subsystems.HeadSubsystem;
import frc.robot.subsystems.LEDSubsystem;

public class LEDManager extends Command {
    
    private final LEDSubsystem leds;
    private final HeadSubsystem head;
    private final ElevSubsystem elev;
    private final CommandXboxController driver1, driver2;
    
    private final LEDDualBreatheAnimation breathe;
    // private final LEDStepsAnimation steps;
    // private final LEDScanAnimation scan;
    // private final LEDPattern rainbow;
    private final LEDPattern hasCoral;
    private final LEDPattern noCoral;
    private final LEDPattern endgame;
    private final LEDPattern branchAligned;

    private boolean endgameStarted;

    public LEDManager(LEDSubsystem ledSub, HeadSubsystem headSub, ElevSubsystem elevSub, CommandXboxController driver1, CommandXboxController driver2) {
        leds = ledSub;
        head = headSub;
        elev = elevSub;
        this.driver1 = driver1;
        this.driver2 = driver2;
        addRequirements(leds);

        endgameStarted = false;

        breathe = new LEDDualBreatheAnimation(LEDConstants.chargeGreen, LEDConstants.chargeGold, 4, 0.2, 0.4);
        // steps = new LEDStepsAnimation(LEDConstants.chargeGreen, LEDConstants.chargeGold, 4, Units.MetersPerSecond.of(Integer.MAX_VALUE));
        // scan = new LEDScanAnimation(LEDConstants.chargeGold, 10, 6);
        // rainbow = LEDPattern.rainbow(255, 255)
        //     .scrollAtAbsoluteSpeed(Units.MetersPerSecond.of(4), LEDConstants.ledSpacing);

        hasCoral = LEDPattern.solid(LEDConstants.white);
        noCoral = LEDPattern.solid(LEDConstants.orange);
        endgame = LEDPattern.rainbow(255, 255)
            .scrollAtAbsoluteSpeed(Units.MetersPerSecond.of(3), LEDConstants.ledSpacing);
        branchAligned = LEDPattern.solid(LEDConstants.chargeGreen).blink(Units.Seconds.of(0.33), Units.Seconds.of(0.33));
    }

    @Override
    public void execute() {
        if(DriverStation.isDisabled()) {
            breathe.update();
            breathe.evaluate(leds.fullBuff());
            // steps.update();
            // steps.evaluate(leds.fullBuff());
            // scan.update();
            // scan.evaluate(leds.fullBuff());
            // rainbow.applyTo(leds.fullBuff());
            return;
        }

        if(DriverStation.isTeleopEnabled()) {
            if(Timer.getMatchTime() <= LEDConstants.endgameSecs && Timer.getMatchTime() >= LEDConstants.endgameSecs - LEDConstants.endgameDisplayTime) {
                endgame.applyTo(leds.fullBuff());
                
                if(!endgameStarted) {
                    driver1.setRumble(RumbleType.kBothRumble, 1);
                    driver2.setRumble(RumbleType.kBothRumble, 1);

                    endgameStarted = true;
                }

                if(Timer.getMatchTime() < LEDConstants.endgameSecs - LEDConstants.endgameRumbleTime && endgameStarted) {
                    driver1.setRumble(RumbleType.kBothRumble, 0);
                    driver2.setRumble(RumbleType.kBothRumble, 0);
                }
                return;
            }
        }

        if(head.getHasCoral()) {
            if(LEDConstants.branchAlignLevels.contains(elev.getPositionLevel()) && head.getBranchSensor(elev.getPositionLevel())) {
                branchAligned.applyTo(leds.fullBuff());
                return;
            }

            hasCoral.applyTo(leds.fullBuff());
            return;
        }

        noCoral.applyTo(leds.fullBuff());
    }

    @Override
    public boolean isFinished() {
        return false;
    }
    @Override
    public boolean runsWhenDisabled() {
        return true;
    }
    //called in disabled init
    public void resetEndgameStarted() {
        endgameStarted = false;
    }
}
