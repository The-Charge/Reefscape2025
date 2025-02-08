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
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
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

    private final CommandXboxController driverXbox = new CommandXboxController(0);
    private final Joystick buttonBox = new Joystick(1);

    private final SwerveSubsystem swerve = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));
    private final VisionSubsystem m_limelight = new VisionSubsystem(swerve);
    // private final ElevSubsystem elev = new ElevSubsystem();
    // private final ClimbSubsystem climb = new ClimbSubsystem();
    // private final HeadSubsystem head = new HeadSubsystem();
    
    /*
     * ======================
     * Auto
     * ======================
     */
    private SendableChooser<Command> autoChooser;
    
    public RobotContainer() {
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
        addTelemetry();
        DriverStation.silenceJoystickConnectionWarning(true);

        autoChooser = AutoBuilder.buildAutoChooser();
        setupAutoDisplay();
        SmartDashboard.putData("Auto Chooser", autoChooser);

        Field2d field = new Field2d();
        SmartDashboard.putData("Field", field);
    }

    private void configureBindings() {
        driverXbox.b().onTrue(Commands.runOnce(swerve::zeroGyro));
        driverXbox.x().whileTrue(Commands.runOnce(swerve::lock, swerve).repeatedly());

        // new Trigger(() -> buttonBox.getRawButton(1)).onTrue(new InstantCommand(elev::stop));
        // new Trigger(() -> buttonBox.getRawButton(2)).onTrue(new MoveToInches(elev, 0));
        // new Trigger(() -> buttonBox.getRawButton(3)).onTrue(new MoveToLevel(elev, ElevSubsystem.Level.LVL1));
        // new Trigger(() -> buttonBox.getRawButton(4)).onTrue(new MoveToLevel(elev, ElevSubsystem.Level.LVL2));
        // new Trigger(() -> buttonBox.getRawButton(5)).onTrue(new MoveToLevel(elev, ElevSubsystem.Level.LVL3));
        // new Trigger(() -> buttonBox.getRawButton(6)).onTrue(new MoveToLevel(elev, ElevSubsystem.Level.LVL4));

        // new Trigger(() -> buttonBox.getRawButton(1)).onTrue(new InstantCommand(climb::stop));
        // new Trigger(() -> buttonBox.getRawButton(2)).onTrue(new Climb(climb));
        // new Trigger(() -> buttonBox.getRawButton(3)).onTrue(new Declimb(climb));

        // new Trigger(() -> buttonBox.getRawButton(1)).onTrue(new Intake(head));
        // new Trigger(() -> buttonBox.getRawButton(2)).onTrue(new Shoot(head));

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

        /*
         * Please use consitent naming conventions!!
         * <Subsytem><Action>
         * Use PascalCase
         */
        // NamedCommands.registerCommand("ElevHome", new MoveToLevel(elev, ElevSubsystem.Level.HOME));
        // NamedCommands.registerCommand("ElevLevel1", new MoveToLevel(elev, ElevSubsystem.Level.LVL1));
        // NamedCommands.registerCommand("ElevLevel2", new MoveToLevel(elev, ElevSubsystem.Level.LVL2));
        // NamedCommands.registerCommand("ElevLevel3", new MoveToLevel(elev, ElevSubsystem.Level.LVL3));
        // NamedCommands.registerCommand("ElevLevel4", new MoveToLevel(elev, ElevSubsystem.Level.LVL4));

        // NamedCommands.registerCommand("HeadIntake", new Intake(head));
        // NamedCommands.registerCommand("HeadShoot", new Shoot(head));
    }
    private void addTelemetry() {
        //one time telemetry values, such as dashboard commands
        // SmartDashboard.putData("Elev Manual Move (IN)", new MoveToInchesManual(elev));
        // SmartDashboard.putData("Elev Manual Move (TICKS)", new MoveToTicksManual(elev));
        // SmartDashboard.putData("Elev Manual Move (LVL)", new MoveToLevelManual(elev));

        // SmartDashboard.putData("Climb Manual Climb (DEG)", new ClimbToDegreesManual(climb));
        // SmartDashboard.putData("Climb Manual Climb (TICKS)", new ClimbToTicksManual(climb));
        // SmartDashboard.putData("Climb Manual", new Climb(climb));
        // SmartDashboard.putData("Declimb Manual", new Declimb(climb));

        // SmartDashboard.putData("Head Intake", new Intake(head));
        // SmartDashboard.putData("Head Shoot", new Shoot(head));
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

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
    public SwerveSubsystem getSwerveSubsystem() {
      return swerve;
    }
}