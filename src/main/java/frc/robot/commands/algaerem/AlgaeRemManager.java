package frc.robot.commands.algaerem;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.constants.AlgaeRemConstants;
import frc.robot.subsystems.AlgaeRemSubsystem;
import frc.robot.subsystems.ElevSubsystem;

public class AlgaeRemManager extends Command {
    
    private final AlgaeRemSubsystem algae;
    private final ElevSubsystem elev;

    public AlgaeRemManager(AlgaeRemSubsystem algaeSub, ElevSubsystem elevSub) {
        algae = algaeSub;
        elev = elevSub;

        addRequirements(algae);
    }

    @Override
    public void execute() {
        ElevSubsystem.Level cLevel = elev.getPositionLevel();
        if(cLevel != ElevSubsystem.Level.ALGAE_LOW && cLevel != ElevSubsystem.Level.ALGAE_HIGH && algae.getSetpointPercent() == AlgaeRemConstants.servoOutPercent) {
            new SequentialCommandGroup(
                new AlgaeRemStop(algae),
                new AlgaeRemIn(algae)
            ).schedule();
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
