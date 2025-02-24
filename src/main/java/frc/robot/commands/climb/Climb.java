package frc.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ClimbConstants;
import frc.robot.subsystems.ClimbSubsystem;

public class Climb extends Command {
    
    private ClimbSubsystem climb;
    private boolean wait;

    public Climb(ClimbSubsystem climbSub) {
        this(climbSub, false);
    }
    public Climb(ClimbSubsystem climbSub, boolean waitForTarget) {
        climb = climbSub;
        wait = waitForTarget;

        addRequirements(climb);
    }

    @Override
    public void initialize() {
        climb.setTargetAngleDegrees(ClimbConstants.activeDegrees);
    }

    @Override
    public boolean isFinished() {
        if(!wait)
            return true;
        
        return climb.isAtTarget();
    }
}
