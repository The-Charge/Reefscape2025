package frc.robot.subsystems;

import static edu.wpi.first.units.Units.DegreesPerSecond;

import java.util.Optional;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.constants.VisionConstants.ApriltagConstants;
import frc.robot.constants.VisionConstants.LLFunnelConstants;
import frc.robot.constants.VisionConstants.LLReefConstants;
import limelight.Limelight;
import limelight.estimator.LimelightPoseEstimator;
import limelight.estimator.PoseEstimate;
import limelight.structures.AngularVelocity3d;
import limelight.structures.LimelightSettings.LEDMode;
import limelight.structures.Orientation3d;

public class VisionSubsystem extends SubsystemBase{
    SwerveSubsystem swerve;

    // private final AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);
   

    public Timer detectiontimer = new Timer();
    public boolean timerstarted = false;

    Limelight reeflimelight;
    Limelight funnellimelight;
    LimelightPoseEstimator funnelPoseEstimator;
    LimelightPoseEstimator reefPoseEstimator;

    

    public double[] tx = new double[2];               //X-offset
    public double[] ty = new double[2];               //Y-offset
    public boolean[] tv = new boolean[2];               //Target Identification
    public double[] ta = new double[2];               //Area of tag
    public double[] tid = new double[2];              //Tag id
    public double[] tl = new double[2];               //latency contribution
    public double[] cl = new double[2];               //Capture pipeline latency
    public double[] getpipe = new double[2];          //get current pipeline
    public double[] limelightlatency = new double[2]; //tl + cl
    public double[] distance = new double[2];         //distance to target
    
    

    public VisionSubsystem(SwerveSubsystem swerve){
      this.swerve = swerve;
      
      //Setup YALL limelight object (maybe should be in initialize)
      reeflimelight = new Limelight(LLReefConstants.LL_NAME);
      reeflimelight.getSettings().withLimelightLEDMode(LEDMode.PipelineControl).withCameraOffset(LLReefConstants.CAMERA_OFFSET /*Can be Pose3d.kZero if it does not cooperate*/).save();
      reefPoseEstimator = reeflimelight.getPoseEstimator(true);

      /*funnellimelight = new Limelight(LLFunnelConstants.LL_NAME);
      funnellimelight.getSettings().withLimelightLEDMode(LEDMode.PipelineControl).withCameraOffset(Pose3d.kZero).save();
       * 
       */
      
    
      
    }

    @Override
    public void periodic(){
      // This method will be called once per scheduler run
      updateLimelightTracking();
      UpdateLocalization();
      double[] stddevs = NetworkTableInstance.getDefault().getTable(LLReefConstants.LL_NAME).getEntry("stddevs")
          .getDoubleArray(new double[] { 0.0 });
      for (int i = 0; i < stddevs.length; i++) {
        SmartDashboard.putNumber("Standard Deviation " + i, stddevs[i]);
      }
    }
  
    //updates limelight tracked values and puts on SmartDashboard
    public void updateLimelightTracking(){ 
        //LimelightHepers.setPipelineIndex(VisionConstants.LLReefConstants.LL_NAME, 1);
        //Read general target values
        
        tv[0] = LimelightHelpers.getTV(LLReefConstants.LL_NAME);
        tx[0] = LimelightHelpers.getTX(LLReefConstants.LL_NAME);
        ty[0] = LimelightHelpers.getTY(LLReefConstants.LL_NAME);
        ta[0] = LimelightHelpers.getTA(LLReefConstants.LL_NAME);
        tid[0] = NetworkTableInstance.getDefault().getTable(LLReefConstants.LL_NAME).getEntry("tid").getDouble(0);
        tl[0] = LimelightHelpers.getLatency_Pipeline(LLReefConstants.LL_NAME);
        cl[0] = LimelightHelpers.getLatency_Capture(LLReefConstants.LL_NAME);
        getpipe[0] = LimelightHelpers.getCurrentPipelineIndex(LLReefConstants.LL_NAME);

        tv[1] = LimelightHelpers.getTV(LLFunnelConstants.LL_NAME);
        tx[1] = LimelightHelpers.getTX(LLFunnelConstants.LL_NAME);
        ty[1] = LimelightHelpers.getTY(LLFunnelConstants.LL_NAME);
        ta[1] = LimelightHelpers.getTA(LLFunnelConstants.LL_NAME);
        tid[1] = NetworkTableInstance.getDefault().getTable(LLFunnelConstants.LL_NAME).getEntry("tid").getDouble(0);
        tl[1] = LimelightHelpers.getLatency_Pipeline(LLFunnelConstants.LL_NAME);
        cl[1] = LimelightHelpers.getLatency_Capture(LLFunnelConstants.LL_NAME);
        getpipe[1] = LimelightHelpers.getCurrentPipelineIndex(LLFunnelConstants.LL_NAME);
    }

    public void setPipeline(double index){
      NetworkTableInstance.getDefault().getTable(LLReefConstants.LL_NAME).getEntry("pipeline").setNumber(index);
    }

    public void adjustDriverPipeline(){
      if (getpipe[0] == 1) 
        NetworkTableInstance.getDefault().getTable(LLReefConstants.LL_NAME).getEntry("pipeline").setNumber(0);
      else
        NetworkTableInstance.getDefault().getTable(LLReefConstants.LL_NAME).getEntry("pipeline").setNumber(1);
    }
   
    public void UpdateLocalization(){
      reeflimelight.getSettings()
      .withRobotOrientation(new Orientation3d(new Rotation3d(0, 0, swerve.getHeading().getRadians()),
                          new AngularVelocity3d(DegreesPerSecond.of(0),
                                      DegreesPerSecond.of(0),
                                      swerve.getSwerveDrive().getGyro().getYawAngularVelocity().copy())))
      .save();
      SmartDashboard.putNumber("swerve rotation", swerve.getRotation3d().getX());
      // Get the vision estimate.
      Optional<PoseEstimate> visionEstimate = reefPoseEstimator.getPoseEstimate(); // BotPose.BLUE_MEGATAG2.get(limelight);
      visionEstimate.ifPresent((PoseEstimate poseEstimate) -> {
      // If the average tag distance is less than 4 meters,
      // there are more than 0 tags in view,
      // and the average ambiguity between tags is less than 50% then we update the pose estimation.
        if (poseEstimate.tagCount > 0 && poseEstimate.getMinTagAmbiguity() < 0.3)
      {
        swerve.addVisionReading(poseEstimate.pose.toPose2d(), poseEstimate.timestampSeconds);
        SmartDashboard.putNumber("TagX", poseEstimate.pose.toPose2d().getX());
        SmartDashboard.putNumber("TagY", poseEstimate.pose.toPose2d().getY());
        SmartDashboard.putNumber("Timestamp", poseEstimate.timestampSeconds);
        SmartDashboard.putNumber("Rotationtag", poseEstimate.pose.toPose2d().getRotation().getDegrees());
        SmartDashboard.putNumber("Distance to tag", poseEstimate.avgTagDist);
        SmartDashboard.putBoolean("Pose Estimated", true);
      }
      else{
        SmartDashboard.putBoolean("Pose Estimated", false);
      }
      SmartDashboard.putNumber("Tag Ambig", poseEstimate.getMinTagAmbiguity());

    });
    }
    
    public boolean robotRotationWithinThreshold(int tagid){
      return 
        Math.abs(ApriltagConstants.TAG_POSES[tagid].getRotation().getAngle() * 180 / Math.PI - 180
        - reefPoseEstimator.getPoseEstimate().get().pose.toPose2d().getRotation().getDegrees())
        < ApriltagConstants.ANGLE_POSE_TOLERANCE;
    }
    public boolean getTV(int index){
      return tv[index];
    }
    public double getTagID(int index){
      return tid[index];
    }
    public double getTX(int index){
      return tx[index];
    }
    public double getTY(int index){
      return ty[index];
    }
    public double getTA(int index){
      return ta[index];
    }

    public double getTargetRotation(int index){
        return LimelightHelpers.getTargetPose3d_RobotSpace(LLReefConstants.LL_NAME).getRotation().getY(); //This is yaw
    }

    
}