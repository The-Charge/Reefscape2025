package frc.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ClimbConstants;
import frc.robot.subsystems.ClimbSubsystem;

public class Climb extends Command {
    
    private ClimbSubsystem climb;
    private boolean startedLever;
    private double leverVbus;

    public Climb(ClimbSubsystem climbSub) {
        this(climbSub, ClimbConstants.leverMaxVBus);
    }
    public Climb(ClimbSubsystem climbSub, double vbus) {
        climb = climbSub;
        leverVbus = vbus;

        addRequirements(climb);
    }

    @Override
    public void initialize() {
        startedLever = false;

        climb.setClampState(ClimbSubsystem.State.ACTIVE);
    }
    @Override
    public void execute() {
        if(climb.isClampIsAtTarget() && !startedLever) {
            climb.leverVBus(leverVbus); //relies on soft limits to stop from going too far
            startedLever = true;
        }
    }
    @Override
    public void end(boolean interrupted) {
        climb.leverStop();
    }

    @Override
    public boolean isFinished() {
        return climb.getLeverLimitSwitch();
        // return climb.getLeverDegrees() >= ClimbConstants.leverActiveDegrees;
    }
}
