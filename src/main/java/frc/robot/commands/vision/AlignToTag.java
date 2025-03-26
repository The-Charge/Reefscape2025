package frc.robot.commands.vision;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.VisionConstants.ApriltagConstants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.VisionSubsystem.ReefPosition;

public class AlignToTag extends Command {
  private final SwerveSubsystem swerve;
  private VisionSubsystem limelight;
  private ReefPosition reefPos;
  private PIDController sideController = new PIDController(5, 0, 0), frontController = new PIDController(5, 0,
      0), rotController = new PIDController(5, 0, 0);

  public AlignToTag(SwerveSubsystem swerve, VisionSubsystem limelight, VisionSubsystem.ReefPosition pos) {
    this.swerve = swerve;
    this.limelight = limelight;
    this.reefPos = pos;
    addRequirements(swerve);
    addRequirements(limelight);
  }

  @Override
  public void execute() {
    double offset;
    switch (reefPos) {
      case LEFT:
        offset = ApriltagConstants.LEFT_ALIGN_OFFSET;
        break;
      case RIGHT:
        offset = ApriltagConstants.RIGHT_ALIGN_OFFSET;
        break;
      default:
        offset = ApriltagConstants.MID_ALIGN_OFFSET;
        break;
    }

    double tx = limelight.getRobotPoseTagSpace().getX();
    double ty = limelight.getRobotPoseTagSpace().getY();
    double rot = limelight.getRobotPoseTagSpace().getRotation().getY();

    double pidtx = sideController.calculate(ty, offset);
    double pidty = frontController.calculate(tx, 1);
    double pidrot = rotController.calculate(-rot, 0);

    SmartDashboard.putNumberArray("Align array", new Double[] { ty, tx, rot, pidtx, pidty, pidrot });

    // ChassisSpeeds alignmentSpeeds = swerve.getTargetSpeeds(pidtx, pidty, new
    // Rotation2d(pidrot));
    swerve.drive(new Translation2d(pidtx, pidty), pidrot, false);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}