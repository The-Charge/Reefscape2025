package frc.robot.commands.swervedrive.drivebase;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.SwerveConstants;
import frc.robot.subsystems.HeadSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class DriveToSourceDist extends Command {
    
    private final SwerveSubsystem swerve;
    private final HeadSubsystem head;
    private Timer timeout;
    private PIDController pid;

    public DriveToSourceDist(SwerveSubsystem swerveSub, HeadSubsystem headSub) {
        swerve = swerveSub;
        head = headSub;

        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        timeout = new Timer();
        timeout.start();

        pid = new PIDController(SwerveConstants.alignPID.kP, SwerveConstants.alignPID.kI, SwerveConstants.alignPID.kD);
        pid.setSetpoint(SwerveConstants.sourceAcceptableDist);

    }
    @Override
    public void execute() {
        double out = pid.calculate(head.getBackDistance());

                SmartDashboard.putNumber("Swerve AlignToSourceDist pid", out);
        SmartDashboard.putNumber("Swerve AlignToSourceDist actual", MathUtil.clamp(out  * SwerveConstants.sourceAlignSpeed - 0.18, -SwerveConstants.sourceAlignSpeed, SwerveConstants.sourceAlignSpeed));

        swerve.drive(new Translation2d(MathUtil.clamp(out * SwerveConstants.sourceAlignSpeed - 0.18, -SwerveConstants.sourceAlignSpeed, SwerveConstants.sourceAlignSpeed), 0), 0, false);
    }
    @Override
    public void end(boolean interrupted) {
        swerve.drive(Translation2d.kZero, 0, false);
    }

    @Override
    public boolean isFinished() {
        return head.getBackDistance() <= SwerveConstants.sourceAcceptableDist
                || timeout.hasElapsed(SwerveConstants.alignTimeout);
    }
}
