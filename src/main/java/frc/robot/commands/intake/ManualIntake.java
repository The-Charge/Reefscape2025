package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.IntakeConstants;
import frc.robot.subsystems.IntakeSubsystem;

public class ManualIntake extends Command {
    
    private final IntakeSubsystem intake;

    public ManualIntake(IntakeSubsystem intakeSub) {
        intake = intakeSub;

        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.vBus(IntakeConstants.intakeVBus);
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
