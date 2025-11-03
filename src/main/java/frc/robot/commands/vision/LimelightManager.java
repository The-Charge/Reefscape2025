package frc.robot.commands.vision;

import static edu.wpi.first.units.Units.DegreesPerSecond;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers.PoseEstimate;
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
        
        PoseEstimate reefEstimate = reefLimelight.getLLHPoseEstimate(yaw, 0);
        PoseEstimate funnelEstimate = funnelLimelight.getLLHPoseEstimate(yaw, 0);
        PoseEstimate reefEstimateTag1 = reefLimelight.getLLHPoseEstimateTag1(yaw, 0);
        PoseEstimate funnelEstimateTag1 = funnelLimelight.getLLHPoseEstimateTag1(yaw, 0);
        Double[] reefstddev = reefLimelight.getSTDDevs();
        Double[] funnelstddev = funnelLimelight.getSTDDevs();

        LoggingManager.logValue("ReefTags", Pose3d.struct, reefLimelight.getTagPose3ds(), true);
        LoggingManager.logValue("FunnelTags", Pose3d.struct, funnelLimelight.getTagPose3ds(), true);
        if (reefEstimate != null) {
            LoggingManager.logAndAutoSendValue("Reef BotX Tag2", reefEstimate.pose.getX());
            LoggingManager.logAndAutoSendValue("Reef BotY Tag2", reefEstimate.pose.getY());
        }
        if (reefEstimateTag1 != null) {
            LoggingManager.logAndAutoSendValue("Reef BotX Tag1", reefEstimateTag1.pose.getX());
            LoggingManager.logAndAutoSendValue("Reef BotY Tag1", reefEstimateTag1.pose.getY());
        }
        if (funnelEstimate != null) {
            LoggingManager.logAndAutoSendValue("Funnel BotX Tag2", funnelEstimate.pose.getX());
            LoggingManager.logAndAutoSendValue("Funnel BotY Tag2", funnelEstimate.pose.getY());
        }
        if (funnelEstimateTag1 != null) {
            LoggingManager.logAndAutoSendValue("Funnel BotX Tag1", funnelEstimateTag1.pose.getX());
            LoggingManager.logAndAutoSendValue("Funnel BotY Tag1", funnelEstimateTag1.pose.getY());
        }
        if (TelemetryConstants.debugTelemetry) {
            SmartDashboard.putBoolean("reef estimated", false);
            SmartDashboard.putBoolean("funnel estimated", false);

            SmartDashboard.putNumberArray("reef stddevs", reefstddev);
            SmartDashboard.putNumberArray("funnel ambig", funnelstddev);

            SmartDashboard.putNumber("swerve rot speed", yawRate);
        }

        final int big = Integer.MAX_VALUE;

        LoggingManager.logAndAutoSendValue("Tag1 rotation", false);
        if (reefstddev[5] < funnelstddev[5]) {
            swerve.addVisionReading(reefEstimateTag1.pose, reefEstimateTag1.timestampSeconds,
                    VecBuilder.fill(big, big, reefstddev[5]));
            LoggingManager.logAndAutoSendValue("Tag1 rotation", true);
        } else {
            swerve.addVisionReading(funnelEstimateTag1.pose, funnelEstimateTag1.timestampSeconds,
                    VecBuilder.fill(big, big, funnelstddev[5]));
            LoggingManager.logAndAutoSendValue("Tag1 rotation", true);
        }

        if (reefstddev[6] * reefstddev[7] <= funnelstddev[6] * funnelstddev[7]) {
            if (TelemetryConstants.debugTelemetry)
                SmartDashboard.putBoolean("reef estimated", true);

            swerve.addVisionReading(reefEstimate.pose, reefEstimate.timestampSeconds, VecBuilder.fill(
                    reefstddev[6], reefstddev[7], big));
        } else {
            if (TelemetryConstants.debugTelemetry)
                SmartDashboard.putBoolean("funnel estimated", true);

            swerve.addVisionReading(funnelEstimate.pose, funnelEstimate.timestampSeconds,
                    VecBuilder.fill(funnelstddev[6], funnelstddev[7], big));
        }
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
