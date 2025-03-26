package frc.robot.commands.vision;

import java.util.function.BooleanSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.SwerveConstants;
import frc.robot.constants.TelemetryConstants;
import frc.robot.constants.VisionConstants.ApriltagConstants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem.ReefPosition;

public class DriveToTag extends Command {
  private final SwerveSubsystem swerve;
  private int tagid;
  private BooleanSupplier cancel;
  private boolean reef;
  private boolean tagged;
  private ReefPosition reefPos;
  private Command drivetoPose;

  public DriveToTag(SwerveSubsystem swerve, boolean reef, BooleanSupplier cancel, ReefPosition reefPos) {
    this.swerve = swerve;
    this.reefPos = reefPos;
    this.cancel = cancel;
    this.reef = reef;
    this.tagged = false;
    
    addRequirements(swerve);
  }
  
  public DriveToTag(SwerveSubsystem swerve, int tagid, BooleanSupplier cancel, ReefPosition reefPos) {
    this.swerve = swerve;
    this.reefPos = reefPos;
    this.cancel = cancel;
    this.tagid = tagid;
    this.tagged = true;
    
    addRequirements(swerve);
  }
  
  @Override
  public void initialize() {
    drivetoPose = null;
    double offset = 0;

    if (!tagged) {
      if (reef) {
        tagid = swerve.getClosestTagIDReef();
      } else {
        tagid = swerve.getClosestTagIDStation();
      }
    }

    switch (reefPos) {
      case LEFT:
        offset = ApriltagConstants.LEFT_ALIGN_OFFSET;
        SmartDashboard.putString("side", "LEFT");
        break;
        case RIGHT:
        offset = ApriltagConstants.RIGHT_ALIGN_OFFSET;
        SmartDashboard.putString("side", "Rigth");
        break;
        default:
        offset = ApriltagConstants.MID_ALIGN_OFFSET;
        SmartDashboard.putString("side", "center");
        break;
    }

    double x, y;
    Rotation2d rot2d;
    double rot;

    rot2d = ApriltagConstants.TAG_POSES[tagid].getRotation().toRotation2d();
    rot = rot2d.getRadians();

    x = ApriltagConstants.TAG_POSES[tagid].getX() + 1 * Math.cos(rot) - offset * Math.sin(rot);
    y = ApriltagConstants.TAG_POSES[tagid].getY() + 1 * Math.sin(rot) + offset * Math.cos(rot);

    if (!(tagid == 1 || tagid == 2 || tagid == 12 || tagid == 13)) {
      rot2d = rot2d.minus(Rotation2d.k180deg);
    }

    Pose2d targetPose = new Pose2d(x, y, rot2d);

    PathConstraints constraints = new PathConstraints(SwerveConstants.MAX_SPEED / 2, 1.0, 2 * Math.PI, 4 * Math.PI);

    if (TelemetryConstants.debugTelemetry) {
      SmartDashboard.putNumber("tag x", x);
      SmartDashboard.putNumber("tag y", y);
      SmartDashboard.putNumber("swerve x", swerve.getPose().getX());
      SmartDashboard.putNumber("swerve y", swerve.getPose().getY());
    }

    if (drivetoPose != null) {
      drivetoPose.cancel();
      System.out.println("AHHHHHHHH");
    }
    // drivetoPose = null;

    // run the path
    drivetoPose = AutoBuilder.pathfindToPose(
        targetPose,
        constraints,
        0.0 // Goal end velocity in meters/sec
    );

    drivetoPose.onlyWhile(cancel).schedule();
  }
  
  @Override
  public boolean isFinished() {
    if (drivetoPose != null) {
      return drivetoPose.isFinished();
    }
    return false;
  }

  @Override
  public void end(boolean IntakeRumble) {
    drivetoPose.cancel();
    drivetoPose = null;
  }
}