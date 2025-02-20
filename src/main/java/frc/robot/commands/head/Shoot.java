package frc.robot.commands.head;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.HeadConstants;
import frc.robot.subsystems.HeadSubsystem;

public class Shoot extends Command {

    private HeadSubsystem m_head; 
    private Timer timeout; 

    public Shoot(HeadSubsystem head) {
        this.m_head = head;
        addRequirements(head);
    }

    @Override
    public void initialize() {
        timeout = new Timer(); 
        timeout.start();

        m_head.flywheelVBus(HeadConstants.shootVBus);
    }
    @Override
    public void end(boolean interrupted) {
        m_head.stop();
    }

    @Override
    public boolean isFinished() {
        return timeout.hasElapsed(HeadConstants.shootTime);
    }       
}
