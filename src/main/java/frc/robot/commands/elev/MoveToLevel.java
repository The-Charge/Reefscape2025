package frc.robot.commands.elev;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevSubsystem;
import frc.robot.subsystems.HeadSubsystem;

public class MoveToLevel extends Command {

    private final ElevSubsystem elev;
    private final HeadSubsystem head;
    private final ElevSubsystem.Level targetLevel;
    
    private boolean wait;

    public MoveToLevel(ElevSubsystem elevSub, HeadSubsystem headSub, ElevSubsystem.Level level) {
        this(elevSub, headSub, level, false);
    }
    public MoveToLevel(ElevSubsystem elevSub, HeadSubsystem headSub, ElevSubsystem.Level level, boolean waitForTarget) {
        elev = elevSub;
        head = headSub;
        targetLevel = level;
        wait = waitForTarget;

        addRequirements(elev);
    }

    @Override
    public void initialize() {
        if(targetLevel != ElevSubsystem.Level.HOME && targetLevel != ElevSubsystem.Level.ALGAE_LOW && targetLevel != ElevSubsystem.Level.ALGAE_HIGH && !head.getHasCoral()) {
            wait = false; //prevent the command from getting stuck
            return;
        }

        elev.setTargetPositionLevel(targetLevel);
    }

    @Override
    public boolean isFinished() {
        if(!wait)
            return true;
        
        return elev.isAtTarget();
    }
}
