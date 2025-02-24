// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;
import java.util.Optional;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.climb.Climb;
import frc.robot.commands.climb.ClimbToDegreesManual;
import frc.robot.commands.climb.ClimbToTicksManual;
import frc.robot.commands.climb.Declimb;
import frc.robot.commands.elev.MoveToInchesManual;
import frc.robot.commands.elev.MoveToLevel;
import frc.robot.commands.elev.MoveToLevelManual;
import frc.robot.commands.elev.MoveToTicksManual;
import frc.robot.commands.intake.ManualIntake;
import frc.robot.commands.leds.LEDManager;
import frc.robot.commands.swervedrive.drivebase.SwerveZero;
import frc.robot.commands.swervedrive.drivebase.TeleopDrive;
import frc.robot.commands.vision.LimelightManager;
import frc.robot.constants.SwerveConstants;
import frc.robot.constants.VisionConstants.LLFunnelConstants;
import frc.robot.constants.VisionConstants.LLReefConstants;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.ElevSubsystem;
import frc.robot.subsystems.ElevSubsystem.Level;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

/**
* This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
* little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
* Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
*/
public class RobotContainer {

    private final CommandXboxController driver1 = new CommandXboxController(0);
    private final CommandXboxController driver2 = new CommandXboxController(1);

    private final SwerveSubsystem swerve = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));
    private final VisionSubsystem reeflimelight = new VisionSubsystem(swerve, LLReefConstants.LL_NAME, LLReefConstants.CAMERA_OFFSET);
    private final VisionSubsystem funnellimelight = new VisionSubsystem(swerve, LLFunnelConstants.LL_NAME, LLFunnelConstants.CAMERA_OFFSET);
    private final ElevSubsystem elev = new ElevSubsystem();
    private final ClimbSubsystem climb = new ClimbSubsystem();
    // private final HeadSubsystem head = new HeadSubsystem();
    private final IntakeSubsystem intake = new IntakeSubsystem();
    // private final AlgaeRemSubsystem algaeRem = new AlgaeRemSubsystem();
    private final LEDSubsystem leds = new LEDSubsystem();
    
    private SendableChooser<Command> autoChooser;
    private TeleopDrive teleop;
    
    public RobotContainer() {
        teleop = new TeleopDrive(swerve,
            () -> -MathUtil.applyDeadband(driver1.getLeftY(), SwerveConstants.LEFT_Y_DEADBAND),
            () -> -MathUtil.applyDeadband(driver1.getLeftX(), SwerveConstants.LEFT_X_DEADBAND),
            () -> -MathUtil.applyDeadband(driver1.getRightX(), SwerveConstants.RIGHT_X_DEADBAND),
            () -> driver1.povCenter().getAsBoolean(),
            () -> driver1.povDown().getAsBoolean(), () -> driver1.povDownLeft().getAsBoolean(),
            () -> driver1.povDownRight().getAsBoolean(),
            () -> driver1.povLeft().getAsBoolean(), () -> driver1.povRight().getAsBoolean(),
            () -> driver1.povUp().getAsBoolean(),
            () -> driver1.povUpLeft().getAsBoolean(), () -> driver1.povUpRight().getAsBoolean(),
            () -> driver1.rightBumper().getAsBoolean(),
            () -> driver1.back().getAsBoolean(),
            () -> driver1.leftTrigger(SwerveConstants.TRIGGER_DEADBAND).getAsBoolean(),
            () -> driver1.rightTrigger(SwerveConstants.TRIGGER_DEADBAND).getAsBoolean()
        );

        // intake.setDefaultCommand(new Intake(intake, elev, head));
        leds.setDefaultCommand(new LEDManager(leds));
        
        configureNamedCommands();
        configureBindings();
        addTelemetry();
        DriverStation.silenceJoystickConnectionWarning(true);

        autoChooser = AutoBuilder.buildAutoChooser();
        setupAutoDisplay();
        SmartDashboard.putData("Auto Chooser", autoChooser);

        new LimelightManager(swerve, reeflimelight, funnellimelight).schedule();

        Field2d field = new Field2d();
        SmartDashboard.putData("Field", field);
    }

    private void configureBindings() {
        driver1.b().onTrue(Commands.runOnce(swerve::zeroGyroWithAlliance));
        driver1.x().whileTrue(Commands.runOnce(swerve::lock, swerve).repeatedly());
        
        driver2.a().onTrue(new Climb(climb));
        driver2.y().onTrue(new Declimb(climb));
        driver2.povUp().onTrue(new MoveToLevel(elev, Level.LVL4));
        driver2.povRight().onTrue(new MoveToLevel(elev, Level.LVL3));
        driver2.povLeft().onTrue(new MoveToLevel(elev, Level.LVL2));
        driver2.povDown().onTrue(new MoveToLevel(elev, Level.LVL1));
        driver2.leftTrigger(SwerveConstants.TRIGGER_DEADBAND).onTrue(new MoveToLevel(elev, Level.HOME));
        driver2.x().whileTrue(new ManualIntake(intake));
        // driver2.rightTrigger(SwerveConstants.TRIGGER_DEADBAND).onTrue(new SequentialCommandGroup(
        //     new Shoot(head),
        //     new WaitCommand(3),
        //     new MoveToLevel(elev, Level.HOME)
        // ));
        // driver2.rightBumper().onTrue(new SequentialCommandGroup(
        //     new MoveToLevel(elev, Level.ALGAE_HIGH, true),
        //     new AlgaeRemSpin(algaeRem)
        // ));
        // driver2.leftBumper().onTrue(new SequentialCommandGroup(
        //     new MoveToLevel(elev, Level.ALGAE_LOW, true),
        //     new AlgaeRemSpin(algaeRem)
        // ));

        // new Trigger(() -> head.getFunnelSensor()).onTrue(new Index(head).withInterruptBehavior(InterruptionBehavior.kCancelIncoming)); //we don't want the head to do anything until indexing is finished

        // driverXbox.a().onTrue(Commands.runOnce(swerve::addFakeVisionReading));
        // driverXbox.y().whileTrue(new DriveToTag(swerve, m_limelight, 7));
        // driverXbox.leftBumper().onTrue(Commands.runOnce(m_limelight::adjustDriverPipeline));

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
        NamedCommands.registerCommand("ElevHome", new MoveToLevel(elev, ElevSubsystem.Level.HOME));
        NamedCommands.registerCommand("ElevLevel1", new MoveToLevel(elev, ElevSubsystem.Level.LVL1));
        NamedCommands.registerCommand("ElevLevel2", new MoveToLevel(elev, ElevSubsystem.Level.LVL2));
        NamedCommands.registerCommand("ElevLevel3", new MoveToLevel(elev, ElevSubsystem.Level.LVL3));
        NamedCommands.registerCommand("ElevLevel4", new MoveToLevel(elev, ElevSubsystem.Level.LVL4));

        // NamedCommands.registerCommand("HeadIntake", new Intake(head));
        // NamedCommands.registerCommand("HeadShoot", new Shoot(head));

        // NamedCommands.registerCommand("AlgaeRemIn", new AlgaeRemIn(algaeRem));
        // NamedCommands.registerCommand("AlgaeRemOut", new AlgaeRemOut(algaeRem));
        // NamedCommands.registerCommand("AlgaeRemSpin", new AlgaeRemSpin(algaeRem));
        // NamedCommands.registerCommand("AlgaeRemStop", new AlgaeRemStop(algaeRem));
    }
    private void addTelemetry() {
        //one time telemetry values, such as dashboard commands
        SmartDashboard.putData("Elev Manual Move (IN)", new MoveToInchesManual(elev));
        SmartDashboard.putData("Elev Manual Move (TICKS)", new MoveToTicksManual(elev));
        SmartDashboard.putData("Elev Manual Move (LVL)", new MoveToLevelManual(elev));

        SmartDashboard.putData("Climb Manual Climb (DEG)", new ClimbToDegreesManual(climb));
        SmartDashboard.putData("Climb Manual Climb (TICKS)", new ClimbToTicksManual(climb));
        SmartDashboard.putData("Climb Manual", new Climb(climb));
        SmartDashboard.putData("Declimb Manual", new Declimb(climb));

        // SmartDashboard.putData("Head Intake", new Intake(head));
        // SmartDashboard.putData("Head Shoot", new Shoot(head));

        // SmartDashboard.putData("AlgaeRem In", new AlgaeRemIn(algaeRem));
        // SmartDashboard.putData("AlgaeRem Out", new AlgaeRemOut(algaeRem));
        // SmartDashboard.putData("AlgaeRem Spin", new AlgaeRemSpin(algaeRem));
        // SmartDashboard.putData("AlgaeRem Stop", new AlgaeRemStop(algaeRem));
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
    public void setTeleopDefaultCommand() {
        swerve.setDefaultCommand(teleop);
    }
    public void clearTeleopDefaultCommand() {
        swerve.setDefaultCommand(new SwerveZero(swerve));
    }
}