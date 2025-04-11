package frc.robot.commands.vision;

import static edu.wpi.first.units.Units.DegreesPerSecond;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.commands.LoggingManager;
import frc.robot.constants.TelemetryConstants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class LimelightManager extends Command {
    private final SwerveSubsystem swerve;
    private VisionSubsystem reefLimelight;
    private VisionSubsystem funnelLimelight;

    public LimelightManager(SwerveSubsystem swerve, VisionSubsystem reefLimelight, VisionSubsystem funnnelLimelight) {
        this.swerve = swerve;
        this.reefLimelight = reefLimelight;
        this.funnelLimelight = funnnelLimelight;
    }

    @Override
    public void initialize() {
        LoggingManager.logAndAutoSendValue("LimelightManager running", true);
    }
    @Override
    public void execute() {
        double yaw = swerve.getHeading().getDegrees();
        double yawRate = Math.abs(swerve.getSwerveDrive().getGyro().getYawAngularVelocity().in(DegreesPerSecond));
        double speed = Math.hypot(
                swerve.getFieldVelocity().vxMetersPerSecond,
                swerve.getFieldVelocity().vyMetersPerSecond
        );

        LimelightHelpers.PoseEstimate reefEstimate = reefLimelight.getLLHPoseEstimate(yaw, 0);
        LimelightHelpers.PoseEstimate funnelEstimate = funnelLimelight.getLLHPoseEstimate(yaw, 0);
        LimelightHelpers.PoseEstimate reefEstimateTag1 = reefLimelight.getLLHPoseEstimateTag1(yaw, 0);
        LimelightHelpers.PoseEstimate funnelEstimateTag1 = funnelLimelight.getLLHPoseEstimateTag1(yaw, 0);
        // LimelightHelpers.PoseEstimate funnelEstimate = null;

        // Pose2d reefPose = reefLimelight.getEstimatedPose(yaw, yawRate);
        // Pose2d funnelPose = funnelLimelight.getEstimatedPose(yaw, yawRate);

        // double reefTime = reefLimelight.getPoseTimestamp();
        // double funnelTime = funnelLimelight.getPoseTimestamp();

        double reefAmbig = reefLimelight.getAmbiguity();
        double funnelAmbig = funnelLimelight.getAmbiguity();

        int reefCount = reefLimelight.getTagCount();
        int funnelCount = funnelLimelight.getTagCount();

        LoggingManager.logValue("ReefTags", Pose3d.struct, reefLimelight.getTagPose3ds(), true);
        LoggingManager.logValue("FunnelTags", Pose3d.struct, funnelLimelight.getTagPose3ds(), true);
        if (reefEstimate != null) {
            LoggingManager.logAndAutoSendValue("Reef BotX Tag2", reefEstimate.pose.getX());
            LoggingManager.logAndAutoSendValue("Reef BotY Tag2", reefEstimate.pose.getY());
        }
        if(reefEstimateTag1 != null) {
            LoggingManager.logAndAutoSendValue("Reef BotX Tag1", reefEstimateTag1.pose.getX());
            LoggingManager.logAndAutoSendValue("Reef BotY Tag1", reefEstimateTag1.pose.getY());
        }
        if (funnelEstimate != null) {
            LoggingManager.logAndAutoSendValue("Funnel BotX Tag2", funnelEstimate.pose.getX());
            LoggingManager.logAndAutoSendValue("Funnel BotY Tag2", funnelEstimate.pose.getY());
        }
        if(funnelEstimateTag1 != null) {
            LoggingManager.logAndAutoSendValue("Funnel BotX Tag1", funnelEstimateTag1.pose.getX());
            LoggingManager.logAndAutoSendValue("Funnel BotY Tag1", funnelEstimateTag1.pose.getY());
        }
        if(TelemetryConstants.debugTelemetry) {
            SmartDashboard.putBoolean("reef estimated", false);
            SmartDashboard.putBoolean("funnel estimated", false);

            SmartDashboard.putNumber("reef ambig", reefAmbig);
            SmartDashboard.putNumber("funnel ambig", funnelAmbig);

            SmartDashboard.putNumber("swerve rot speed", yawRate);
        }

        boolean reefEstim = (reefEstimate != null && reefCount > 0 && yawRate < Units.degreesToRadians(60)); // reefAmbig < 0.2 ||
        boolean funnelEstim = (funnelEstimate != null && funnelCount > 0 && yawRate < Units.degreesToRadians(60)); // funnelAmbig < 0.2 ||

        LoggingManager.logAndAutoSendValue("Tag1 rotation", false);
        if ((reefAmbig < 0.2 || reefCount > 1) && yawRate < Units.degreesToRadians(36) && speed < 0.5) {
            swerve.addVisionReading(reefEstimateTag1.pose, reefEstimateTag1.timestampSeconds, VecBuilder.fill(9999999,9999999,2));
            LoggingManager.logAndAutoSendValue("Tag1 rotation", true);
        }
        
        if ((funnelAmbig < 0.2 || funnelCount > 1) && yawRate < Units.degreesToRadians(36) && speed < 0.5) {
            swerve.addVisionReading(funnelEstimateTag1.pose, funnelEstimateTag1.timestampSeconds, VecBuilder.fill(9999999,9999999,2));
            LoggingManager.logAndAutoSendValue("Tag1 rotation", true);
        }

        if (reefEstim && funnelEstim) {
            if (reefCount > funnelCount) {
                if(TelemetryConstants.debugTelemetry)
                    SmartDashboard.putBoolean("reef estimated", true);
                
                swerve.addVisionReading(reefEstimate.pose, reefEstimate.timestampSeconds, VecBuilder.fill(.7,.7,9999999));
            } else if (reefCount < funnelCount) {
                if(TelemetryConstants.debugTelemetry)
                    SmartDashboard.putBoolean("funnel estimated", true);

                swerve.addVisionReading(funnelEstimate.pose, funnelEstimate.timestampSeconds, VecBuilder.fill(.7,.7,9999999));
            } else if (reefAmbig < funnelAmbig) {
                if(TelemetryConstants.debugTelemetry)
                    SmartDashboard.putBoolean("reef estimated", true);

                swerve.addVisionReading(reefEstimate.pose, reefEstimate.timestampSeconds, VecBuilder.fill(.7,.7,9999999));
            } else {
                if(TelemetryConstants.debugTelemetry)
                    SmartDashboard.putBoolean("funnel estimated", true);

                swerve.addVisionReading(funnelEstimate.pose, funnelEstimate.timestampSeconds, VecBuilder.fill(.7,.7,9999999));
            }
        } else if (reefEstim) {
            if(TelemetryConstants.debugTelemetry)
                SmartDashboard.putBoolean("reef estimated", true);

            swerve.addVisionReading(reefEstimate.pose, reefEstimate.timestampSeconds, VecBuilder.fill(.7,.7,9999999));
        } else if (funnelEstim) {
            if(TelemetryConstants.debugTelemetry)
                SmartDashboard.putBoolean("funnel estimated", true);
                
            swerve.addVisionReading(funnelEstimate.pose, funnelEstimate.timestampSeconds, VecBuilder.fill(.7,.7,9999999));
        }

        // Rotation2d swerveRot = swerve.getHeading();

        // if (reefPose == null && funnelPose == null) {
        //     return;
        // } else if (reefPose == null) {
        //     avgPose = funnelPose;
        // } else if (funnelPose == null) {
        //     avgPose = reefPose;
        // } else {
        //     double avgX = (reefPose.getX() + funnelPose.getX()) / 2;
        //     double avgY = (reefPose.getY() + funnelPose.getY()) / 2;
        //     // Rotation2d avgRot = reefPose.getRotation().plus(funnelPose.getRotation()).times(0.5);

        //     avgPose = new Pose2d(avgX, avgY, swerveRot);
        // }    

        // if (Double.isNaN(reefTime) && Double.isNaN(funnelTime)) {
        //     System.err.println("Limelight could not get timestamp.");
        // } else if (Double.isNaN(reefTime)) {
        //     avgTime = funnelTime;
        // } else if (Double.isNaN(funnelTime)) {
        //     avgTime = reefTime;
        // } else {
        //     avgTime = (reefTime + funnelTime) / 2;
        // }

        // swerve.addVisionReading(avgPose, avgTime);
    }
    @Override
    public void end(boolean interrupted) {
        LoggingManager.logAndAutoSendValue("LimelightManager running", false);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
