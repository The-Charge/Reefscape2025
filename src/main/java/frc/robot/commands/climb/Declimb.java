package frc.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ClimbConstants;
import frc.robot.subsystems.ClimbSubsystem;

public class Declimb extends Command {
    
    private ClimbSubsystem climb;
    private boolean wait;
    private boolean canRun;

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
        canRun = climb.getLeverDegrees() <= ClimbConstants.leverSafeDegrees;

        if(canRun) {
            climb.setLeverState(ClimbSubsystem.State.RESTING);
            climb.setClampState(ClimbSubsystem.State.RESTING);
        }
    }

    @Override
    public boolean isFinished() {
        if(!wait || !canRun)
            return true;
        
        return climb.isLeverIsAtTarget() && climb.isClampIsAtTarget();
    }
}
