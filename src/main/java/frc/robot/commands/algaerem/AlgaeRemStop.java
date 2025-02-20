package frc.robot.commands.algaerem;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeRemSubsystem;

public class AlgaeRemStop extends Command {
    
    private final AlgaeRemSubsystem algaeRem;

    public AlgaeRemStop(AlgaeRemSubsystem algaeRemSub) {
        algaeRem = algaeRemSub;

        addRequirements(algaeRem);
    }

    @Override
    public void initialize() {
        algaeRem.setFlywheelVBus(0);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
