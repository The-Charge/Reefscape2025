package frc.robot.commands.elev;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevSubsystem;
import frc.robot.subsystems.ElevSubsystem.Level;

public class AdjustL4Override extends Command {
    
    private final ElevSubsystem elev;
    private final double offset;

    public AdjustL4Override(ElevSubsystem elevSub, double delta) {
        elev = elevSub;
        offset = delta;

        addRequirements(elev);
    }

    @Override
    public void execute() {
        elev.setL4Override(elev.getL4Override() + offset);
        elev.setTargetPositionLevel(Level.LVL4);
    }
    
    @Override
    public boolean isFinished() {
        return false; //made for use wth a trigger whileTrue
    }
}
