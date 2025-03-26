package frc.robot.commands.vision;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.SwerveConstants;
import frc.robot.constants.VisionConstants.ApriltagConstants;
import frc.robot.constants.VisionConstants.LLReefConstants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class DriveToAlgae extends Command {
    private final SwerveSubsystem swerve;
    private final VisionSubsystem limelight;
    private final PIDController heading_controller;
    private final PIDController drive_controller;
    private Timer detectiontimer;
    private boolean timerstarted = false;
    
    public DriveToAlgae(SwerveSubsystem swerve, VisionSubsystem limelight){
        this.swerve = swerve;
        this.limelight = limelight;
        addRequirements(swerve);
        addRequirements(limelight);
        heading_controller = new PIDController(5, 0.0, 0.0);
        heading_controller.setTolerance(ApriltagConstants.TX_TOLERANCE);
        heading_controller.setSetpoint(0.0);

        drive_controller = new PIDController(5, 0, 0);
        drive_controller.setTolerance(ApriltagConstants.TARGET_DISTANCE_TOLERANCE);
        drive_controller.setSetpoint(0.0);

        detectiontimer = new Timer();
    }

    @Override
    public void initialize() {
        limelight.setPipeline(LLReefConstants.ALGAE_PIPELINE);
    }

    @Override
    public void execute() {
        heading_controller.reset();
        drive_controller.reset();
        double RotationVal = MathUtil.clamp(heading_controller.calculate(limelight.getTX(), 0.0), -1, 1);
        double TranslationVal = MathUtil.clamp(drive_controller.calculate(limelight.getDistToCamera(), 0.0), -0.5, 0.5);

        if (limelight.getObjectedDetected()){
            timerstarted = false;
        if (detectiontimer != null){
            detectiontimer.stop();
        }
      
        swerve.drive(new Translation2d(-1 * TranslationVal * SwerveConstants.MAX_SPEED,0), RotationVal * swerve.getSwerveController().config.maxAngularVelocity, false);
        }   

        if (!limelight.getObjectedDetected()){
            if (!timerstarted){
                detectiontimer = new Timer();
                detectiontimer.start();
                timerstarted = true;
            } 

        }
        if (detectiontimer != null){
            SmartDashboard.putNumber("detectiontimer", detectiontimer.get());
        }
    }

    @Override
    public void end(boolean interrupted) {
        detectiontimer = new Timer();
        limelight.setPipeline(LLReefConstants.APRILTAG_PIPELINE);
    }

    @Override
    public boolean isFinished() {
        if (detectiontimer == null){
            return false;
        }  
        else{
            return detectiontimer.hasElapsed(0.2);
        }
    }

}