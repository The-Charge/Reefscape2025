package frc.robot.commands.leds;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.leds.patterns.LEDDualBreatheAnimation;
import frc.robot.commands.leds.patterns.LEDScanAnimation;
import frc.robot.commands.leds.patterns.LEDStepsAnimation;
import frc.robot.constants.LEDConstants;
import frc.robot.subsystems.HeadSubsystem;
import frc.robot.subsystems.LEDSubsystem;

public class LEDManager extends Command {
    
    private final LEDSubsystem leds;
    private final HeadSubsystem head;
    private final CommandXboxController driver1, driver2;
    
    private final LEDDualBreatheAnimation breathe;
    private final LEDStepsAnimation steps;
    private final LEDScanAnimation scan;
    private final LEDPattern hasCoral;
    private final LEDPattern noCoral;
    private final LEDPattern endgame;

    private final double disabledAnimationTime = 10; //length each disabled animation displays for
    private final Timer disabledTimer;

    private final double endgameSecs = 20; //seconds left for endgame
    private final double endgameDisplayTime = 20; //amount of time the endgame pattern is displayed for
    private final double endgameRumbleTime = 2; //amount of time the controllers rumble for

    private boolean endgameStarted;
    private int currentDisabledAnimation;

    public LEDManager(LEDSubsystem ledSub, HeadSubsystem headSub, CommandXboxController driver1, CommandXboxController driver2) {
        leds = ledSub;
        head = headSub;
        this.driver1 = driver1;
        this.driver2 = driver2;
        addRequirements(leds);

        endgameStarted = false;
        currentDisabledAnimation = 0;
        disabledTimer = new Timer();
        disabledTimer.start();

        breathe = new LEDDualBreatheAnimation(LEDConstants.chargeGreen, LEDConstants.chargeGold, 4, 0.2, 0.4);
        steps = new LEDStepsAnimation(LEDConstants.chargeGreen, LEDConstants.chargeGold, 4, Units.MetersPerSecond.of(1));
        scan = new LEDScanAnimation(LEDConstants.chargeGold, 20, 4);
        hasCoral = LEDPattern.solid(LEDConstants.white);
        noCoral = LEDPattern.solid(LEDConstants.orange);
        endgame = LEDPattern.rainbow(255, 255)
            .scrollAtAbsoluteSpeed(Units.MetersPerSecond.of(3), LEDConstants.ledSpacing);
    }

    @Override
    public void execute() {
        if(DriverStation.isDisabled()) {
            if(disabledTimer.advanceIfElapsed(disabledAnimationTime)) {
                currentDisabledAnimation = (currentDisabledAnimation + 1) % 3;
            }

            switch(currentDisabledAnimation) {
                default:
                case 0:
                    breathe.update();
                    breathe.evaluate(leds.fullBuff());
                    break;
                
                case 1:
                    steps.update();
                    steps.evaluate(leds.fullBuff());
                    break;
                
                case 2:
                    scan.update();
                    scan.evaluate(leds.fullBuff());
                    break;
            }
            return;
        }

        if(DriverStation.isTeleopEnabled()) {
            if(Timer.getMatchTime() <= endgameSecs && Timer.getMatchTime() >= endgameSecs - endgameDisplayTime) {
                endgame.applyTo(leds.fullBuff());
                
                if(!endgameStarted) {
                    driver1.setRumble(RumbleType.kBothRumble, 1);
                    driver2.setRumble(RumbleType.kBothRumble, 1);

                    endgameStarted = true;
                }

                if(Timer.getMatchTime() < endgameSecs - endgameRumbleTime && endgameStarted) {
                    driver1.setRumble(RumbleType.kBothRumble, 0);
                    driver2.setRumble(RumbleType.kBothRumble, 0);
                }
                return;
            }

            if(head.getHasCoral()) {
                hasCoral.applyTo(leds.fullBuff());
                return;
            }
    
            noCoral.applyTo(leds.fullBuff());
        }
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
