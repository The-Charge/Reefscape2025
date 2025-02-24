package frc.robot.commands.elev;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ElevConstants;
import frc.robot.subsystems.ElevSubsystem;

public class MoveToInchesManual extends Command {
    
    private ElevSubsystem elev;

    public MoveToInchesManual(ElevSubsystem elevSub) {
        elev = elevSub;
    }

    @Override
    public void initialize() {
        new MoveToInches(elev, SmartDashboard.getNumber(ElevConstants.overrideInName, ElevConstants.minPosTicks * ElevConstants.tickToInConversion), false).schedule();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
