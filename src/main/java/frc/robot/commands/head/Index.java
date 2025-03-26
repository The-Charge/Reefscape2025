package frc.robot.commands.head;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.HeadConstants;
import frc.robot.subsystems.HeadSubsystem;

public class Index extends Command {

    private HeadSubsystem head; 

    public Index(HeadSubsystem headSub) {
        this.head = headSub;

        addRequirements(head);
    }

    @Override 
    public void initialize() {
        head.flywheelLeft(HeadConstants.indexerVbus);
        head.flywheelRight(HeadConstants.indexerVbus);
    }

    @Override 
    public void end(boolean interrupted) {
        head.stop();
    }

    @Override
    public boolean isFinished() {
        return head.getCoralSensor();
    }

}
