package frc.robot.commands.elev;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevSubsystem;

public class MoveToInches extends Command {

    private ElevSubsystem elev;
    private double targetInches;
    private boolean wait;

    public MoveToInches(ElevSubsystem elevSub, double inches) {
        this(elevSub, inches, false);
    }
    public MoveToInches(ElevSubsystem elevSub, double inches, boolean waitForTarget) {
        elev = elevSub;
        targetInches = inches;
        wait = waitForTarget;

        addRequirements(elev);
    }

    @Override
    public void initialize() {
        elev.setTargetPositionInches(targetInches);
    }

    @Override
    public boolean isFinished() {
        if(!wait)
            return true;
        
        return elev.isAtTarget();
    }
}
