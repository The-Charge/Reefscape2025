package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.HeadSubsystem;

public class IntakeRumble extends Command {
    
    private final HeadSubsystem head;
    private final CommandXboxController driver1, driver2;

    private boolean hasCoralLast;
    private Timer rumbleTimer;

    private final double rumbleTime = 0.75; //secs

    public IntakeRumble(HeadSubsystem headSub, CommandXboxController driver1, CommandXboxController driver2) {
        head = headSub;
        this.driver1 = driver1;
        this.driver2 = driver2;
    }

    @Override
    public void initialize() {
        hasCoralLast = true; //don't rumble when the robot is enabled with a coral already in it
        rumbleTimer = null;
    }
    @Override
    public void execute() {
        boolean hasCoral = head.getHasCoral();

        if(hasCoral && !hasCoralLast) {
            driver1.setRumble(RumbleType.kBothRumble, 1);
            driver2.setRumble(RumbleType.kBothRumble, 1);

            rumbleTimer = new Timer();
            rumbleTimer.start();
        }
        else if(rumbleTimer != null && rumbleTimer.hasElapsed(rumbleTime)) {
            driver1.setRumble(RumbleType.kBothRumble, 0);
            driver2.setRumble(RumbleType.kBothRumble, 0);

            rumbleTimer = null;
        }

        hasCoralLast = hasCoral;
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
