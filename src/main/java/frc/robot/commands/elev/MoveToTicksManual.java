package frc.robot.commands.elev;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ElevConstants;
import frc.robot.subsystems.ElevSubsystem;

public class MoveToTicksManual extends Command {
    
    private ElevSubsystem elev;

    public MoveToTicksManual(ElevSubsystem elevSub) {
        elev = elevSub;
    }

    @Override
    public void initialize() {
        new MoveToTicks(elev, SmartDashboard.getNumber(ElevConstants.overrideTicksName, ElevConstants.minPosTicks), false).schedule();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
