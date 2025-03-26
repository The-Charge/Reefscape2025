package frc.robot.commands.climb;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ClimbConstants;
import frc.robot.subsystems.ClimbSubsystem;

public class ClimbLeverDegreesManual extends Command {
    
    private ClimbSubsystem climb;

    public ClimbLeverDegreesManual(ClimbSubsystem climbSub) {
        climb = climbSub;
    }

    @Override
    public void initialize() {
        new ClimbLeverDegrees(climb, SmartDashboard.getNumber(ClimbConstants.leverOverrideDegName, ClimbConstants.leverRestingDegrees), false).schedule();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
