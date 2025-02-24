package frc.robot.commands.elev;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevSubsystem;

public class MoveToLevel extends Command {

    private ElevSubsystem elev;
    private ElevSubsystem.Level targetLevel;
    private boolean wait;

    public MoveToLevel(ElevSubsystem elevSub, ElevSubsystem.Level level) {
        this(elevSub, level, false);
    }
    public MoveToLevel(ElevSubsystem elevSub, ElevSubsystem.Level level, boolean waitForTarget) {
        elev = elevSub;
        targetLevel = level;
        wait = waitForTarget;

        addRequirements(elev);
    }

    @Override
    public void initialize() {
        elev.setTargetPositionLevel(targetLevel);
    }

    @Override
    public boolean isFinished() {
        if(!wait)
            return true;
        
        return elev.isAtTarget();
    }
}
