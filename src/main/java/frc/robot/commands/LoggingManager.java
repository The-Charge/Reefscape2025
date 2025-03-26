package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.ElevSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class LoggingManager extends Command {
    
    private final SwerveSubsystem swerve;
    private final VisionSubsystem reef;
    private final VisionSubsystem funnel;
    private final ElevSubsystem elev;
    private final ClimbSubsystem climb;

    private final StructPublisher<Pose2d> posePublisher;
    private final StructArrayPublisher<Pose3d> reefTagPublisher;
    private final StructArrayPublisher<Pose3d> funnelTagPublisher;
    private final StructPublisher<Pose3d> elevPublisher;
    private final StructPublisher<Pose3d> leverPublisher;
    private final StructPublisher<Pose3d> clampPublisher;
    private final StructPublisher<Pose3d> bumperPublisher;

    /**
     * Note: Doesn't require any subsystems, just needs them for querying data
     */
    public LoggingManager(SwerveSubsystem swerveSub, VisionSubsystem reefSub, VisionSubsystem funnelSub, ElevSubsystem elevSub, ClimbSubsystem climbSub) {
        swerve = swerveSub;
        reef = reefSub;
        funnel = funnelSub;
        elev = elevSub;
        climb = climbSub;

        //I never close these because they should only be closed when the robot is turned off, and any memory leaks will be deleted by RAM shutting down
        posePublisher = NetworkTableInstance.getDefault().getStructTopic("AvScope/SwervePose", Pose2d.struct).publish();
        reefTagPublisher = NetworkTableInstance.getDefault().getStructArrayTopic("AvScope/ReefTags", Pose3d.struct).publish();
        funnelTagPublisher = NetworkTableInstance.getDefault().getStructArrayTopic("AvScope/FunnelTags", Pose3d.struct).publish();
        elevPublisher = NetworkTableInstance.getDefault().getStructTopic("AvScope/ElevPose", Pose3d.struct).publish();
        leverPublisher = NetworkTableInstance.getDefault().getStructTopic("AvScope/LeverPose", Pose3d.struct).publish();
        clampPublisher = NetworkTableInstance.getDefault().getStructTopic("AvScope/ClampPose", Pose3d.struct).publish();
        bumperPublisher = NetworkTableInstance.getDefault().getStructTopic("AvScope/BumperPose", Pose3d.struct).publish();
    }

    @Override
    public void initialize() {
        bumperPublisher.set(Pose3d.kZero);
    }
    @Override
    public void execute() {
        posePublisher.set(swerve.getPose());
        
        reefTagPublisher.set(reef.getTagPose3ds());
        funnelTagPublisher.set(funnel.getTagPose3ds());

        elevPublisher.set(new Pose3d(new Translation3d(0, 0, elev.getPositionInches() / 39.37), Rotation3d.kZero));

        leverPublisher.set(new Pose3d(Translation3d.kZero, new Rotation3d(0, climb.getLeverDegrees() * Math.PI / 180, 0)));
        clampPublisher.set(new Pose3d(Translation3d.kZero, new Rotation3d(0, 0, -climb.getClampDegrees() * Math.PI / 180)));
    }

    @Override
    public boolean isFinished() {
        return false;
    }
    @Override
    public boolean runsWhenDisabled() {
        return true;
    }
}
