package frc.robot;

import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public abstract class AutoDisplayHelper {

    public static void displayAutoPath(Command autoCommand, boolean isRedAlliance) {

        //The name is "InstantCommand" when Command.none() is passed
        if(autoCommand == null || autoCommand.getName().equals("InstantCommand")) {
            DriverStation.reportError("Cannot display the path of a null/empty command, use clearAutoPath instead", true);
            return;
        }

        List<PathPlannerPath> paths;
        try {
            paths = PathPlannerAuto.getPathGroupFromAutoFile(autoCommand.getName());
        }
        catch(Exception e) {
            e.printStackTrace();
            return;
        }

        //generate each trajectory and merge them into one
        Trajectory mergedTraj = null;
        for(PathPlannerPath path : paths) {
            if(isRedAlliance)
                path = path.flipPath();

            List<Pose2d> poses = getPosesFromPath(path);
            Trajectory traj = generateTrajectory(poses);

            if(mergedTraj == null) {
                mergedTraj = traj;
                continue;
            }
            mergedTraj = mergedTraj.concatenate(traj);
        }

        if(mergedTraj == null) {
            DriverStation.reportWarning("AutoDisplayHelper::displayAutoPath -> Auto has no paths, redirecting to clearAutoPath", false);
            clearAutoPath();
            return;
        }
        
        /*
         * Push field2d
         * For a reason that not even god knows you have to push a blank field then set the trajectory after in order for it to update
         */
        Field2d field = new Field2d();
        
        SmartDashboard.putData("Field", field);
        field.getObject("traj").setTrajectory(mergedTraj);
    }
    public static void displayTelePath(PathPlannerPath path) {
        if (path == null) {clearAutoPath(); return;}
        Field2d field = new Field2d();
        
        List<Pose2d> poses = getPosesFromPath(path);
        if (poses == null) return;
        if (poses.size() < 2) return;
        System.out.println("attempting to add traj");
        Trajectory traj = generateTrajectory(poses);
        if (traj == null) {clearAutoPath(); return;}
        SmartDashboard.putData("Field", field);
        field.getObject("traj").setTrajectory(traj);
    }
    public static void clearAutoPath() {
        System.out.println("clear path");
        Field2d field = new Field2d();

        SmartDashboard.putData("Field", field);
        field.getObject("traj").setTrajectory(new Trajectory());
    }

    private static Trajectory generateTrajectory(List<Pose2d> poses) {
        TrajectoryConfig trajConfig = new TrajectoryConfig(3, 3); //values don't matter as long as they aren't zero
        if (poses == null) return null;
        if (poses.size() < 2) return null;
        return TrajectoryGenerator.generateTrajectory(poses, trajConfig);
    }
    private static Rotation2d calculatePoseRotation(PathPoint current, PathPoint next) {
        //calculate rotation from current point to next point using atan2
        double rad = Math.atan2(next.position.getY() - current.position.getY(), next.position.getX() - current.position.getX());
        return new Rotation2d(rad);
    }

    private static List<Pose2d> getPosesFromPath(PathPlannerPath path) {
        List<PathPoint> pathPoints = new ArrayList<>(path.getAllPathPoints());
        List<Pose2d> poses = new ArrayList<>();
        Translation2d pose = new Translation2d();
        for (int i = 0; i < pathPoints.size(); i++) {
            PathPoint pp = pathPoints.get(i);
            pose = pp.position;
            Rotation2d rot;
            if(i == 0) {
                if(pathPoints.size() > 1)
                    rot = calculatePoseRotation(pp, pathPoints.get(i + 1)); //For the first point, use the heading towards the second point
                else
                    rot = new Rotation2d(); //If there's only one point, use a default rotation
            } else if(i == pathPoints.size() - 1)
                rot = calculatePoseRotation(pathPoints.get(i - 1), pp); //For the last point, use the heading from the second-to-last point
            else {
                // For intermediate points, calculate the heading as the average of the direction to the next and previous points
                Rotation2d toNext = calculatePoseRotation(pp, pathPoints.get(i + 1));
                Rotation2d fromPrev = calculatePoseRotation(pathPoints.get(i - 1), pp);
                rot = new Rotation2d((toNext.getRadians() + fromPrev.getRadians()) / 2.0);
            }
            if (i > 0 && pose.getX() == poses.get(i-1).getX() && pose.getY() == poses.get(i-1).getY()) {
                System.out.println("same point :(");
                poses.remove(i-1);
                pathPoints.remove(i);
                poses.add(new Pose2d(pose, rot));
                i--;
            } else poses.add(new Pose2d(pose, rot));
        }

        return poses;
    }
}
