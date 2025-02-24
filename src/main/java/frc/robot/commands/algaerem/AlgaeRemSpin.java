package frc.robot.commands.algaerem;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.AlgaeRemConstants;
import frc.robot.subsystems.AlgaeRemSubsystem;

public class AlgaeRemSpin extends Command {
    
    private final AlgaeRemSubsystem algaeRem;
    private final Timer timeout;

    public AlgaeRemSpin(AlgaeRemSubsystem algaeRemSub) {
        algaeRem = algaeRemSub;
        timeout = new Timer();

        addRequirements(algaeRem);
    }

    @Override
    public void initialize() {
        timeout.start();
        algaeRem.setFlywheelVBus(AlgaeRemConstants.flywheelActiveVbus);
    }
    @Override
    public void end(boolean interrupted) {
        algaeRem.stop();
    }

    @Override
    public boolean isFinished() {
        return timeout.hasElapsed(AlgaeRemConstants.spinTime);
    }
}
