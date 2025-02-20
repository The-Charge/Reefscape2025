package frc.robot.commands.algaerem;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.AlgaeRemConstants;
import frc.robot.subsystems.AlgaeRemSubsystem;

public class AlgaeRemOut extends Command {
    
    AlgaeRemSubsystem algaeRem;

    public AlgaeRemOut(AlgaeRemSubsystem algaeRemSub) {
        algaeRem = algaeRemSub;

        addRequirements(algaeRem);
    }

    @Override
    public void initialize() {
        algaeRem.goToPercent(AlgaeRemConstants.servoOutPercent);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
