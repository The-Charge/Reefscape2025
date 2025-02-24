package frc.robot.commands.elev;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevSubsystem;

public class MoveToLevelManual extends Command {
    
    private ElevSubsystem elev;

    public MoveToLevelManual(ElevSubsystem elevSub) {
        elev = elevSub;
    }

    @Override
    public void initialize() {
        new MoveToLevel(elev, elev.getOverrideLevel(), false).schedule();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
