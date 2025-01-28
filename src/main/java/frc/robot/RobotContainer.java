// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;
import java.util.Optional;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.Rumble;
import frc.robot.commands.swervedrive.drivebase.TeleopDrive;
import frc.robot.constants.SwerveConstants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

/**
* This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
* little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
* Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
*/
public class RobotContainer {
  private final Field2d field;

    
    private final CommandXboxController driverXbox = new CommandXboxController(0);
    private final SwerveSubsystem swerve = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));
    private final VisionSubsystem m_limelight = new VisionSubsystem(swerve);
    // int rotationXboxAxis = 4;
    
    /*
     * ======================
     * Auto
     * ======================
     */
    private SendableChooser<Command> autoChooser;
    
    public RobotContainer() {
        // if (RobotBase.isSimulation()) {
        //     rotationXboxAxis = 2;
        // }
        
        TeleopDrive teleopDrive = new TeleopDrive(swerve,
            () -> -MathUtil.applyDeadband(driverXbox.getLeftY(), SwerveConstants.LEFT_Y_DEADBAND),
            () -> -MathUtil.applyDeadband(driverXbox.getLeftX(), SwerveConstants.LEFT_X_DEADBAND),
            () -> -MathUtil.applyDeadband(driverXbox.getRightX(), SwerveConstants.RIGHT_X_DEADBAND),
            () -> driverXbox.povUp().getAsBoolean(),
            () -> driverXbox.povLeft().getAsBoolean(),
            () -> driverXbox.povDown().getAsBoolean(),
            () -> driverXbox.povRight().getAsBoolean(),
            () -> driverXbox.back().getAsBoolean(),
            () -> driverXbox.leftTrigger(SwerveConstants.TRIGGER_DEADBAND).getAsBoolean(),
            () -> driverXbox.rightTrigger(SwerveConstants.TRIGGER_DEADBAND).getAsBoolean()
        );
        swerve.setDefaultCommand(teleopDrive);
        
        configureNamedCommands();
        configureBindings();
        DriverStation.silenceJoystickConnectionWarning(true);

        autoChooser = AutoBuilder.buildAutoChooser();
        setupAutoDisplay();
        SmartDashboard.putData("Auto Chooser", autoChooser);

        field = new Field2d();
        SmartDashboard.putData("Field", field);
    }

    private void configureBindings() {
        driverXbox.b().onTrue(Commands.runOnce(swerve::zeroGyroWithAlliance));
        driverXbox.x().whileTrue(Commands.runOnce(swerve::lock, swerve).repeatedly());
        driverXbox.a().whileTrue(new Rumble(driverXbox, RumbleType.kRightRumble, 1).repeatedly());
        driverXbox.y().whileTrue(new Rumble(driverXbox, RumbleType.kLeftRumble, 1).repeatedly());

        //driverXbox.b().whileTrue(
        //    swerve.driveToPose(
        //        new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0)))
        //                        );
        //driverXbox.start().whileTrue(Commands.none());
        //driverXbox.back().whileTrue(Commands.none());
        //driverXbox.leftBumper().whileTrue(Commands.runOnce(swerve::lock, swerve).repeatedly());
        //driverXbox.rightBumper().onTrue(Commands.none());
    }
    private void configureNamedCommands() {
        //Pathplanner named commands
    }
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
    public SwerveSubsystem getSwerveSubsystem() {
      return swerve;
    }
    public void setMotorBrake(boolean brake) {
        swerve.setMotorBrake(brake);
    }
    public void displayAuto() {
        Command auto = autoChooser.getSelected();

        if(auto.getName().equals("InstantCommand")) {
            AutoDisplayHelper.clearAutoPath();
            return;
        }

        boolean isRed = false;
        Optional<Alliance> alliance = DriverStation.getAlliance();

        if(alliance.isPresent() && alliance.get() == Alliance.Red)
            isRed = true;

        AutoDisplayHelper.displayAutoPath(auto, isRed);
    }
    private void setupAutoDisplay() {
        //update the displayed auto path in smartdashboard when ever the selection is changed
        //display is cleared in teleopInit
        autoChooser.onChange((selected) -> {
            if(DriverStation.isTeleopEnabled()) //don't display auton path in teleop
                return;

            displayAuto();
        });

        /*
         * Robot.teleopInit clears the display
         * Robot.autonomousInit redraws the display
         */
    }
}