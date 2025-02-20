// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swervedrive.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.VisionConstants.ApriltagConstants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;


/**
 * An example command that uses an example subsystem.
 */

public class DriveToTag extends Command {
    private final SwerveSubsystem swerve;
    private Command drivetoPose;
    private VisionSubsystem limelight;
    private Pose2d intendedPose;
    private int tagid;
    public DriveToTag(SwerveSubsystem swerve, VisionSubsystem limelight, int tagid){
        this.swerve = swerve;
        this.limelight = limelight;
        this.tagid = tagid;
        addRequirements(swerve);
        addRequirements(limelight);
        
    }
 
@Override
  public void initialize() {
      intendedPose = new Pose2d(
        ApriltagConstants.TAG_POSES[tagid].getX() + ApriltagConstants.APRILTAG_POSE_OFFSET * Math.cos(ApriltagConstants.TAG_POSES[tagid].getRotation().getZ()),
        ApriltagConstants.TAG_POSES[tagid].getY() + ApriltagConstants.APRILTAG_POSE_OFFSET * Math.sin(ApriltagConstants.TAG_POSES[tagid].getRotation().getZ()), 
        new Rotation2d(ApriltagConstants.TAG_POSES[tagid].toPose2d().getRotation().getRadians() - Math.PI)
      );
      SmartDashboard.putNumber("DTP Rotation", new Rotation2d(ApriltagConstants.TAG_POSES[tagid].getZ()).minus(new Rotation2d(Math.PI)).getDegrees());
      SmartDashboard.putNumber("DTP RAW TAG ROTATION", ApriltagConstants.TAG_POSES[tagid].getZ());
      drivetoPose = swerve.driveToPose(intendedPose);
      drivetoPose.schedule();
}

  @Override 
  public void execute() {
    if (limelight.getTagID(0) == tagid && limelight.robotRotationWithinThreshold(tagid)){
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetoPose.cancel();
    SmartDashboard.putBoolean("DriveToTag Scheduled", false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return drivetoPose != null;
  }

}