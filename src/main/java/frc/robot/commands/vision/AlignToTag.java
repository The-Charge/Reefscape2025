package frc.robot.commands.vision;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.LimelightHelpers;
import frc.robot.constants.VisionConstants;
import frc.robot.constants.VisionConstants.ApriltagConstants;
import frc.robot.constants.VisionConstants.LLReefConstants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import limelight.Limelight;

public class AlignToTag extends Command {
    private final SwerveSubsystem swerve;
    private VisionSubsystem limelight;
    private int tagid; 
    private double sideVal;
    private double frontVal;
    private ChassisSpeeds alignmentSpeeds;
    private Pose2d tagpose;
    private Rotation2d tagpose_rotation;
    private final PIDController translation_controller;
    private double tx;
    public AlignToTag(SwerveSubsystem swerve, VisionSubsystem limelight, int tagid){
        this.swerve = swerve;
        this.limelight = limelight;
        this.tagid = tagid;
        translation_controller = new PIDController(1, 0.0, 0.0);
        translation_controller.setTolerance(ApriltagConstants.TRANSLATION_POSE_TOLERANCE);
        translation_controller.setSetpoint(0.0);

        
        addRequirements(swerve);
        addRequirements(limelight);
    }
 
  @Override
  public void initialize() {
   
  }

  @Override 
  public void execute() {
    
    tagpose = ApriltagConstants.TAG_POSES[tagid].toPose2d();
    tagpose_rotation = tagpose.getRotation().minus(new Rotation2d(Math.PI));

    translation_controller.reset();
    tx = LimelightHelpers.getTX(limelight.getName());
    sideVal = 14.5 / translation_controller.calculate(tx,0);
    frontVal = 14.5 / translation_controller.calculate(limelight.getDistToCamera(), ApriltagConstants.APRILTAG_POSE_OFFSET);
    swerve.drive(new Translation2d(sideVal, frontVal), 0, true);
    //alignmentSpeeds = swerve.getTargetSpeeds(sideVal, frontVal, tagpose_rotation.getSin(), tagpose_rotation.getCos());
    SmartDashboard.putNumber(
      "sideval", sideVal);
    SmartDashboard.putNumber("frontval", frontVal);
    SmartDashboard.putNumber("tx thing", tx);
    //swerve.drive(alignmentSpeeds);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}