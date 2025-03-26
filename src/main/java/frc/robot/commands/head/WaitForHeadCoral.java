package frc.robot.commands.head;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.HeadSubsystem;

public class WaitForHeadCoral extends Command {
    
    private final HeadSubsystem head;

    /**
     * NOTE: Doesn't require head sub, just needs it to read values
     */
    public WaitForHeadCoral(HeadSubsystem headSub) {
        head = headSub;
    }
    
    @Override
    public boolean isFinished() {
        return head.getHasCoral();
    }
}
