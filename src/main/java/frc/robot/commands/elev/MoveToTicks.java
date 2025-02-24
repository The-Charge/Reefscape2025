package frc.robot.commands.elev;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevSubsystem;

public class MoveToTicks extends Command {

    private ElevSubsystem elev;
    private double targetTicks;
    private boolean wait;

    public MoveToTicks(ElevSubsystem elevSub, double ticks) {
        this(elevSub, ticks, false);
    }
    public MoveToTicks(ElevSubsystem elevSub, double ticks, boolean waitForTarget) {
        elev = elevSub;
        targetTicks = ticks;
        wait = waitForTarget;

        addRequirements(elev);
    }

    @Override
    public void initialize() {
        elev.setTargetPositionTicks(targetTicks);
    }

    @Override
    public boolean isFinished() {
        if(!wait)
            return true;
        
        return elev.isAtTarget();
    }
}
