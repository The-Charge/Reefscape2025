package frc.robot.commands.algaerem;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.AlgaeRemConstants;
import frc.robot.subsystems.AlgaeRemSubsystem;

public class AlgaeRemSpin extends Command {
    
    private final AlgaeRemSubsystem algaeRem;

    public AlgaeRemSpin(AlgaeRemSubsystem algaeRemSub) {
        algaeRem = algaeRemSub;

        addRequirements(algaeRem);
    }

    @Override
    public void initialize() {
        algaeRem.setFlywheelVBus(AlgaeRemConstants.flywheelActiveVbus);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
