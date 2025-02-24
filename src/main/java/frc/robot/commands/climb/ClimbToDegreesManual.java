package frc.robot.commands.climb;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ClimbConstants;
import frc.robot.subsystems.ClimbSubsystem;

public class ClimbToDegreesManual extends Command {
    
    private ClimbSubsystem climb;

    public ClimbToDegreesManual(ClimbSubsystem climbSub) {
        climb = climbSub;
    }

    @Override
    public void initialize() {
        new ClimbToDegrees(climb, SmartDashboard.getNumber(ClimbConstants.overrideDegName, ClimbConstants.restingDegrees), false).schedule();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
