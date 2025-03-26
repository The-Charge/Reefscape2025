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

public class DriveToReefDist extends Command {
    
    private final SwerveSubsystem swerve;
    private final HeadSubsystem head;
    private Timer timeout;
    private PIDController pid;
    private int acumulator = 0;
    private double prev;

    public DriveToReefDist(SwerveSubsystem swerveSub, HeadSubsystem headSub) {
        swerve = swerveSub;
        head = headSub;

        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        timeout = new Timer();
        timeout.start();

        prev = head.getFrontDistance();
        acumulator = 0;

        pid = new PIDController(SwerveConstants.alignPID.kP, SwerveConstants.alignPID.kI, SwerveConstants.alignPID.kD);
        pid.setSetpoint(SwerveConstants.reefAcceptableDist);
    }
    @Override
    public void execute() {
        double out = -pid.calculate(head.getFrontDistance());
        SmartDashboard.putNumber("Swerve AlignToReefDist pid", out);
        SmartDashboard.putNumber("Swerve AlignToReefDist actual", MathUtil.clamp(out  * SwerveConstants.reefAlignSpeed + 0.18, -SwerveConstants.reefAlignSpeed, SwerveConstants.reefAlignSpeed));
        swerve.drive(new Translation2d(MathUtil.clamp(out * SwerveConstants.reefAlignSpeed + 0.18,
                -SwerveConstants.reefAlignSpeed, SwerveConstants.reefAlignSpeed), 0), 0, false);
        if (Math.abs(head.getFrontDistance() - prev) < 20) {
            acumulator++;
        } else {
            acumulator = 0;
        }
        prev = head.getFrontDistance();
    }
    @Override
    public void end(boolean interrupted) {
        swerve.drive(Translation2d.kZero, 0, false);
    }

    @Override
    public boolean isFinished() {
        return head.getFrontDistance() <= SwerveConstants.reefAcceptableDist
                || timeout.hasElapsed(SwerveConstants.alignTimeout) || acumulator > 32;
    }
}
