package frc.robot.constants;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;

public abstract class VisionConstants {

  public static class LLReefConstants{
    public static final String LL_NAME = "limelight-reef";
    public static final double ARRAY_NUM = 0;

    //pipelines
    public static final int VIEWPORT_PIPELINE = 0;
    public static final int APRILTAG_PIPELINE = 1;
    public static final int ALGAE_PIPELINE = 2;

    //limelight pose in robotspace (in meters/radians) - Change later
    public static final double X_CAMERA_OFFSET = Units.inchesToMeters(14); //forward
    public static final double Y_CAMERA_OFFSET = Units.inchesToMeters(5); //right
    public static final double Z_CAMERA_OFFSET = 0.28; //up
    public static final double ROLL_CAMERA_OFFSET = Units.degreesToRadians(0); //3 degree
    public static final double YAW_CAMERA_OFFSET = Units.degreesToRadians(-6.5);
    public static final double PITCH_CAMERA_OFFSET = Units.degreesToRadians(0);
    public static final Pose3d CAMERA_OFFSET = new Pose3d(X_CAMERA_OFFSET, Y_CAMERA_OFFSET, Z_CAMERA_OFFSET, 
                                               new Rotation3d(ROLL_CAMERA_OFFSET, PITCH_CAMERA_OFFSET, YAW_CAMERA_OFFSET));                              
  }

  public static class LLFunnelConstants{
    public static final String LL_NAME = "limelight-funnel";
    public static final double ARRAY_NUM = 1;

    //pipelines
    public static final double VIEWPORT_PIPELINE = 0;
    public static final double APRILTAG_PIPELINE = 1;

    //limelight pose in robotspace (in meters/radian) - Change later
    public static final double X_CAMERA_OFFSET = -0.342425; //forward 0.0
    public static final double Y_CAMERA_OFFSET = -0.03; //right
    public static final double Z_CAMERA_OFFSET = 0.6385; //up
    public static final double ROLL_CAMERA_OFFSET = Units.degreesToRadians(-2.5); //0 degree
    public static final double YAW_CAMERA_OFFSET = Units.degreesToRadians(180); //180 degrees
    public static final double PITCH_CAMERA_OFFSET = Units.degreesToRadians(17.5); //10 degrees
    public static final Pose3d CAMERA_OFFSET = new Pose3d(X_CAMERA_OFFSET, Y_CAMERA_OFFSET, Z_CAMERA_OFFSET, 
                                               new Rotation3d(ROLL_CAMERA_OFFSET, PITCH_CAMERA_OFFSET, YAW_CAMERA_OFFSET));

  }

  public static class ApriltagConstants{ //Apriltag + general constants for vision
    public static final AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
  
    public static final double TRANSLATION_SIDE_POSE_TOLERANCE = 2;
    public static final double TRANSLATION_FRONT_POSE_TOLERANCE = 0.2;
    public static final double TX_TOLERANCE = 5; //degrees
    public static final double TARGET_DISTANCE_TOLERANCE = 0.1; //meters
    
    public static final double ANGLE_POSE_TOLERANCE = 10; //degrees
    public static final double APRILTAG_POSE_OFFSET = 1; //meters

    public static final double LEFT_ALIGN_OFFSET = -0.4; // scoring pipes are +/- 6.47 inches from center of reef tags
    public static final double MID_ALIGN_OFFSET = 0;
    public static final double RIGHT_ALIGN_OFFSET = -0.05;

    //Tag ids for each tag
    public static final double LOW_RED_CORALSTATION_ID = 1;
    public static final double HIGH_RED_CORALSTATION_ID = 2;
    public static final double BLUE_PROCESSOR_ID = 3;
    public static final double RIGHT_BLUE_BARGE_ID = 4;
    public static final double RIGHT_RED_BARGE_ID = 5;
    public static final double BOTTOMRIGHT_RED_REEF_ID = 6;
    public static final double RIGHT_RED_REEF_ID = 7;
    public static final double TOPRIGHT_RED_REEF_ID = 8;
    public static final double TOPLEFT_RED_REEF_ID = 9;
    public static final double LEFT_RED_REEF_ID = 10;
    public static final double BOTTOMLEFT_RED_REEF_ID = 11;
    public static final double BOTTOM_BLUE_CORALSTATION_ID = 12;
    public static final double TOP_BLUE_CORALSTATION_ID = 13;
    public static final double LEFT_BLUE_BARGE_ID = 14;
    public static final double LEFT_RED_BARGE_ID = 15;
    public static final double RED_PROCESSOR_TAG_ID = 16;
    public static final double BOTTOMLEFT_BLUE_REEF_ID = 17;
    public static final double LEFT_BLUE_REEF_ID = 18;
    public static final double TOPLEFT_BLUE_REEF_ID = 19;
    public static final double TOPRIGHT_BLUE_REEF_ID = 20;
    public static final double RIGHT_BLUE_REEF_ID = 21;
    public static final double BOTTOMRIGHT_BLUE_REEF_ID = 22;

    //I am terribly sorry for the sin committed below (pun intended)
    //note: sin is y, cos is x
    
    public static final Pose3d[] TAG_POSES = {
      Pose3d.kZero,
      aprilTagFieldLayout.getTagPose(1).get(),
      aprilTagFieldLayout.getTagPose(2).get(),
      aprilTagFieldLayout.getTagPose(3).get(),
      aprilTagFieldLayout.getTagPose(4).get(),
      aprilTagFieldLayout.getTagPose(5).get(),
      aprilTagFieldLayout.getTagPose(6).get(),
      aprilTagFieldLayout.getTagPose(7).get(),
      aprilTagFieldLayout.getTagPose(8).get(),
      aprilTagFieldLayout.getTagPose(9).get(),
      aprilTagFieldLayout.getTagPose(10).get(),
      aprilTagFieldLayout.getTagPose(11).get(),
      aprilTagFieldLayout.getTagPose(12).get(),
      aprilTagFieldLayout.getTagPose(13).get(),
      aprilTagFieldLayout.getTagPose(14).get(),
      aprilTagFieldLayout.getTagPose(15).get(),
      aprilTagFieldLayout.getTagPose(16).get(),
      aprilTagFieldLayout.getTagPose(17).get(),
      aprilTagFieldLayout.getTagPose(18).get(),
      aprilTagFieldLayout.getTagPose(19).get(),
      aprilTagFieldLayout.getTagPose(20).get(),
      aprilTagFieldLayout.getTagPose(21).get(),
      aprilTagFieldLayout.getTagPose(22).get(),
    };

  }
}
