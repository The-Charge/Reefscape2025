package frc.robot.commands.elev;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevSubsystem;

public class ElevManualUp extends Command {
    
    private final ElevSubsystem elev;

    public ElevManualUp(ElevSubsystem elevSub) {
        elev = elevSub;

        addRequirements(elev);
    }

    @Override
    public void initialize() {
        elev.setVbus(0.3);
    }
    @Override
    public void end(boolean interrupted) {
        elev.stop();
    }

    @Override
    public boolean isFinished() {
        return false; //use with whileTrue
    }
}
