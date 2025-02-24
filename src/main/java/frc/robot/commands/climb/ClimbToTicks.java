package frc.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimbSubsystem;

public class ClimbToTicks extends Command {

    private ClimbSubsystem climb;
    private double targetTicks;
    private boolean wait;

    public ClimbToTicks(ClimbSubsystem climbSub, double ticks) {
        this(climbSub, ticks, false);
    }
    public ClimbToTicks(ClimbSubsystem climbSub, double ticks, boolean waitForTarget) {
        climb = climbSub;
        targetTicks = ticks;
        wait = waitForTarget;

        addRequirements(climb);
    }

    @Override
    public void initialize() {
        climb.setTargetAngleTicks(targetTicks);
    }

    @Override
    public boolean isFinished() {
        if(!wait)
            return true;
        
        return climb.isAtTarget();
    }
}
