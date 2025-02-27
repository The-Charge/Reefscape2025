package frc.robot.commands.head;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.HeadConstants;
import frc.robot.subsystems.ElevSubsystem;
import frc.robot.subsystems.HeadSubsystem;

public class Shoot extends Command {

    private final HeadSubsystem head;
    private final ElevSubsystem elev;
    private Timer timeout; 

    public Shoot(HeadSubsystem headSub, ElevSubsystem elevSub) {
        this.head = headSub;
        this.elev = elevSub;
        addRequirements(head);
    }

    @Override
    public void initialize() {
        timeout = new Timer(); 

        if(elev.getPositionLevel() == ElevSubsystem.Level.LVL1) {
            head.flywheelLeft(HeadConstants.shootVBus / 2);
        }
        else {
            head.flywheelLeft(HeadConstants.shootVBus);
        }
        head.flywheelRight(HeadConstants.shootVBus);
    }
    @Override
    public void execute() {
        if(!head.getHasCoral() && !timeout.isRunning()) {
            timeout.start();
        }
    }
    @Override
    public void end(boolean interrupted) {
        head.stop();
    }

    @Override
    public boolean isFinished() {
        return timeout.hasElapsed(HeadConstants.shootTime);
    }       
}
