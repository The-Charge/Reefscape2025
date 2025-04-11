package frc.robot.commands.swervedrive.drivebase;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.SwerveConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class DriveSlow extends Command {

    private final SwerveSubsystem swerve;
    private final boolean right;

    public DriveSlow(SwerveSubsystem swerveSub, boolean rightBool) {
        swerve = swerveSub;
        right = rightBool;

        addRequirements(swerve);
    }

    @Override
    public void execute() {
        swerve.drive(new Translation2d(0, SwerveConstants.branchAlignSpeed * (right ? -1 : 1)), 0, false);
    }

    @Override
    public void end(boolean interrupted) {
        swerve.drive(Translation2d.kZero, 0, false);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
