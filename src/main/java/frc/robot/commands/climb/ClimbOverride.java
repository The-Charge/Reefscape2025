package frc.robot.commands.climb;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ClimbConstants;
import frc.robot.subsystems.ClimbSubsystem;

public class ClimbOverride extends Command {
    
    private ClimbSubsystem climb;
    private Timer timeout;

    /**
     * ALWAYS HAVE A HAND ON DISABLE WHEN USING. IT WILL BREAK THE CLIMBER
     */
    public ClimbOverride(ClimbSubsystem climbSub) {
        climb = climbSub;

        addRequirements(climb);
    }

    @Override
    public void initialize() {
        timeout = new Timer();
        timeout.start();

        climb.leverVBus(ClimbConstants.leverMaxVBus);
    }
    @Override
    public void end(boolean interrupted) {
        climb.leverStop();
    }

    @Override
    public boolean isFinished() {
        return timeout.hasElapsed(0.5);
    }
}
