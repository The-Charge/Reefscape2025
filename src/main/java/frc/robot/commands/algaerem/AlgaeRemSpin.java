package frc.robot.commands.algaerem;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.AlgaeRemConstants;
import frc.robot.subsystems.AlgaeRemSubsystem;

public class AlgaeRemSpin extends Command {
    
    private final AlgaeRemSubsystem algaeRem;
    private final boolean useTimer;
    
    private Timer timeout;

    public AlgaeRemSpin(AlgaeRemSubsystem algaeRemSub, boolean useTimerStop) {
        algaeRem = algaeRemSub;
        useTimer = useTimerStop;

        addRequirements(algaeRem);
    }

    @Override
    public void initialize() {
        timeout = new Timer();
        timeout.start();

        algaeRem.setFlywheelVBus(AlgaeRemConstants.spinVBus);
    }
    @Override
    public void end(boolean interrupted) {
        algaeRem.stop();
    }

    @Override
    public boolean isFinished() {
        return useTimer ? timeout.hasElapsed(AlgaeRemConstants.spinTime) : false;
    }
}
