package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.IntakeConstants;
import frc.robot.subsystems.ElevSubsystem;
import frc.robot.subsystems.HeadSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class Intake extends Command {
    
    private final IntakeSubsystem intake;
    private final ElevSubsystem elev;
    private final HeadSubsystem head;
    private ElevSubsystem.Level lastLevel = ElevSubsystem.Level.UNKNOWN;
    private boolean lastHadCoral = false;

    public Intake(IntakeSubsystem intakeSub, ElevSubsystem elevSub, HeadSubsystem headSub) {
        intake = intakeSub;
        elev = elevSub;
        head = headSub;

        addRequirements(intake);
    }

    @Override
    public void execute() {
        ElevSubsystem.Level level = elev.getPositionLevel();
        boolean hasCoral = head.getHasCoral();

        if(level != lastLevel || hasCoral != lastHadCoral) {
            switch(level) {
                case HOME:
                    if(!hasCoral)
                        intake.vBus(IntakeConstants.intakeVBus);
                    else
                        intake.stop();
                    break;

                default:
                    intake.stop();
                    break;
            }
        }

        lastLevel = level;
        lastHadCoral = hasCoral;
    }

    @Override
    public void end(boolean interrupted) {
        intake.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
