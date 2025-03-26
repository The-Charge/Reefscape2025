package frc.robot.commands.climb;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ClimbConstants;
import frc.robot.subsystems.ClimbSubsystem;

public class ClimbClampDegreesManual extends Command {
    
    private ClimbSubsystem climb;

    public ClimbClampDegreesManual(ClimbSubsystem climbSub) {
        climb = climbSub;
    }

    @Override
    public void initialize() {
        new ClimbClampDegrees(climb, SmartDashboard.getNumber(ClimbConstants.clampOverrideDegName, ClimbConstants.clampRestingDegrees), false).schedule();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
