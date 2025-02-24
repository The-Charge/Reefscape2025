package frc.robot.commands.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
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
    public boolean runsWhenDisabled() {
        return true;
    }

    @Override
    public void initialize() {
        System.out.println("LimelightManager Initialize");
    }
    
    @Override
    public void execute() {
        Pose2d avgPose;
        Pose2d reefPose = reefLimelight.getEstimatedPose();
        Pose2d funnelPose = funnelLimelight.getEstimatedPose();

        if (reefPose == null && funnelPose == null) {
            return;
        } else if (reefPose == null) {
            avgPose = funnelPose;
        } else if (funnelPose == null) {
            avgPose = reefPose;
        } else {
            double avgX = (reefPose.getX() + funnelPose.getX()) / 2;
            double avgY = (reefPose.getY() + funnelPose.getY()) / 2;
            Rotation2d avgRot = reefPose.getRotation().plus(funnelPose.getRotation()).times(0.5);

            avgPose = new Pose2d(avgX, avgY, avgRot);
        }

        double avgTime = Double.NaN;
        double reefTime = reefLimelight.getPoseTimestamp();
        double funnelTime = funnelLimelight.getPoseTimestamp();

        if (Double.isNaN(reefTime) && Double.isNaN(funnelTime)) {
            System.err.println("Limelight could not get timestamp.");
        } else if (Double.isNaN(reefTime)) {
            avgTime = funnelTime;
        } else if (Double.isNaN(funnelTime)) {
            avgTime = reefTime;
        } else {
            avgTime = (reefTime + funnelTime) / 2;
        }

        swerve.addVisionReading(avgPose, avgTime);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
