package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.HeadSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class WaitForIntakeCoral extends Command {
    
    private final HeadSubsystem head;
    private final IntakeSubsystem intake;
    private Timer timer;

    /**
     * NOTE: Doesn't require head sub, just needs it to read values
     */
    public WaitForIntakeCoral(HeadSubsystem headSub, IntakeSubsystem intakeSub) {
        head = headSub;
        intake = intakeSub;
    }
    
    @Override
    public void initialize() {
        timer = new Timer();
    }
    @Override
    public void execute() {
        if(intake.getVelocity() > 5600)
            timer.start();
        
    }
    @Override
    public boolean isFinished() {
        return head.getHasCoral() || (intake.getAmps() > 7 && timer.hasElapsed(1));
    }
}
