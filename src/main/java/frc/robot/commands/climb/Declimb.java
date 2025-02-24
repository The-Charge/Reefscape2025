package frc.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ClimbConstants;
import frc.robot.subsystems.ClimbSubsystem;

public class Declimb extends Command {
    
    private ClimbSubsystem climb;
    private boolean wait;

    public Declimb(ClimbSubsystem climbSub) {
        this(climbSub, false);
    }
    public Declimb(ClimbSubsystem climbSub, boolean waitForTarget) {
        climb = climbSub;
        wait = waitForTarget;

        addRequirements(climb);
    }

    @Override
    public void initialize() {
        climb.setTargetAngleDegrees(ClimbConstants.restingDegrees);
    }

    @Override
    public boolean isFinished() {
        if(!wait)
            return true;
        
        return climb.isAtTarget();
    }
}
