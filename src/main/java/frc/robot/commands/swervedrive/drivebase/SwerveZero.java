package frc.robot.commands.swervedrive.drivebase;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;

public class SwerveZero extends Command {
    
    private final SwerveSubsystem swerve;

    public SwerveZero(SwerveSubsystem swerveSub) {
        swerve = swerveSub;

        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        swerve.drive(new ChassisSpeeds(0, 0, 0));
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
