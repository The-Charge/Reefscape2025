package frc.robot.commands.climb;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ClimbConstants;
import frc.robot.subsystems.ClimbSubsystem;

public class ClimbToTicksManual extends Command {
    
    private ClimbSubsystem climb;

    public ClimbToTicksManual(ClimbSubsystem climbSub) {
        climb = climbSub;
    }

    @Override
    public void initialize() {
        new ClimbToTicks(climb, SmartDashboard.getNumber(ClimbConstants.overrideTicksName, ClimbConstants.restingDegrees / ClimbConstants.tickToDegConversion), false).schedule();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
