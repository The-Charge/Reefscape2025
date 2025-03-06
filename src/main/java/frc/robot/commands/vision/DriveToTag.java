package frc.robot.commands.vision;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.AutoDisplayHelper;
import frc.robot.constants.SwerveConstants;
import frc.robot.constants.TelemetryConstants;
import frc.robot.constants.VisionConstants.ApriltagConstants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.VisionSubsystem.ReefPosition;

public class DriveToTag extends Command {
  private final SwerveSubsystem swerve;
  private Command drivetoPose;
  private int tagid;
  private boolean reef;
  private ReefPosition reefPos;
  
  public DriveToTag(SwerveSubsystem swerve, boolean reef, VisionSubsystem.ReefPosition reefPos){
      this.swerve = swerve;
      this.reef = reef;
      this.reefPos = reefPos;

      addRequirements(swerve);
  }
 
  @Override
  public void initialize() {
    double offset = 0;

    if (reef) {
      tagid = swerve.getClosestTagIDReef();
    } else {
      tagid = swerve.getClosestTagIDStation();
    }

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
    double x, y;
    Rotation2d rot2d;
    double rot;

    rot2d = ApriltagConstants.TAG_POSES[tagid].getRotation().toRotation2d();
    rot = rot2d.getRadians();

    
    x = ApriltagConstants.TAG_POSES[tagid].getX() + (Units.inchesToMeters(15))*Math.cos(rot) - (ApriltagConstants.CENTER_TO_SCORER_OFFSET + offset)*Math.sin(rot);
    y = ApriltagConstants.TAG_POSES[tagid].getY() + (Units.inchesToMeters(15))*Math.sin(rot) + (ApriltagConstants.CENTER_TO_SCORER_OFFSET + offset)*Math.cos(rot); 
    
    if (!(tagid == 1 || tagid == 2 || tagid == 12 || tagid == 13)) {
      rot2d = rot2d.minus(Rotation2d.k180deg);
    }
    
    SmartDashboard.putNumber("rotation", rot);
    SmartDashboard.putNumber("x vert", 15*Math.cos(rot));
    SmartDashboard.putNumber("y vert", 15*Math.sin(rot));
    SmartDashboard.putNumber("x horiz", -15*Math.sin(rot));
    SmartDashboard.putNumber("y horiz", 15*Math.cos(rot));

    List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
      swerve.getPose(),
      new Pose2d(x, y, rot2d)
    );

    PathConstraints constraints = new PathConstraints(SwerveConstants.MAX_SPEED, 3.0, 2 * Math.PI, 4 * Math.PI);

    // Create the path using the waypoints created above
    PathPlannerPath path = new PathPlannerPath(waypoints, constraints, null, new GoalEndState(0.0, rot2d));

    // Prevent the path from being flipped if the coordinates are already correct
    path.preventFlipping = true;

    if (TelemetryConstants.visionLevel >= TelemetryConstants.EYE_OF_SAURON) {
      SmartDashboard.putNumber("tag x", x);
      SmartDashboard.putNumber("tag y", y);
      SmartDashboard.putNumber("swerve x", swerve.getPose().getX());
      SmartDashboard.putNumber("swerve y", swerve.getPose().getY());
    }
    // run the path
    AutoDisplayHelper.displayTelePath(path);
    drivetoPose = AutoBuilder.followPath(path).withInterruptBehavior(InterruptionBehavior.kCancelSelf);
    drivetoPose.schedule();
  }

  @Override
  public void end(boolean interrupted) {
    if (drivetoPose != null) {
      drivetoPose.cancel();
    }
  }

  @Override
  public boolean isFinished() {
    if (drivetoPose == null) return false;
    return drivetoPose.isFinished();
  }

  public Command getDriveToPose() {
  
    return drivetoPose;
  }
}