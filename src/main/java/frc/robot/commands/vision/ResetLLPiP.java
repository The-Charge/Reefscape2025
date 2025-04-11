package frc.robot.commands.vision;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.VisionSubsystem;

public class ResetLLPiP extends Command {
    
    private final VisionSubsystem ll;

    public ResetLLPiP(VisionSubsystem limelightSub) {
        ll = limelightSub;
    }

    @Override
    public void initialize() {
        ll.setPiP();
    }
    
    @Override
    public boolean isFinished() {
        return true;
    }
    @Override
    public boolean runsWhenDisabled() {
        return true;
    }
}
