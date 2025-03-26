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
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.IntakeRumble;
import frc.robot.commands.LoggingManager;
import frc.robot.commands.algaerem.AlgaeRemSpin;
import frc.robot.commands.climb.Climb;
import frc.robot.commands.climb.ClimbClampDegreesManual;
import frc.robot.commands.climb.ClimbLeverDegreesManual;
import frc.robot.commands.climb.Declimb;
import frc.robot.commands.elev.MoveToInchesManual;
import frc.robot.commands.elev.MoveToLevel;
import frc.robot.commands.elev.MoveToLevelManual;
import frc.robot.commands.elev.MoveToTicksManual;
import frc.robot.commands.head.Shoot;
import frc.robot.commands.head.ShootSlow;
import frc.robot.commands.head.WaitForHeadCoral;
import frc.robot.commands.intake.Intake;
import frc.robot.commands.intake.ManualIntake;
import frc.robot.commands.intake.WaitForIntakeCoral;
import frc.robot.commands.leds.LEDManager;
import frc.robot.commands.swervedrive.drivebase.AlignToBranch;
import frc.robot.commands.swervedrive.drivebase.DriveToReefDist;
import frc.robot.commands.swervedrive.drivebase.DriveToSourceDist;
import frc.robot.commands.swervedrive.drivebase.SwerveZero;
import frc.robot.commands.swervedrive.drivebase.TeleopDrive;
import frc.robot.commands.vision.AlignToTag;
import frc.robot.commands.vision.DriveToTag;
import frc.robot.commands.vision.LimelightManager;
import frc.robot.constants.ClimbConstants;
import frc.robot.constants.SwerveConstants;
import frc.robot.constants.TelemetryConstants;
import frc.robot.constants.VisionConstants.LLFunnelConstants;
import frc.robot.constants.VisionConstants.LLReefConstants;
import frc.robot.subsystems.AlgaeRemSubsystem;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.ElevSubsystem;
import frc.robot.subsystems.ElevSubsystem.Level;
import frc.robot.subsystems.HeadSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.VisionSubsystem.ReefPosition;

/**
* This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
* little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
* Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
*/
public class RobotContainer {

    private final CommandXboxController driver1;
    private final CommandXboxController driver2;
    private final XboxController hid1, hid2;
    
    private final SwerveSubsystem swerve = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));
    private final VisionSubsystem reeflimelight = new VisionSubsystem(LLReefConstants.LL_NAME, LLReefConstants.CAMERA_OFFSET, true);
    private final VisionSubsystem funnellimelight = new VisionSubsystem(LLFunnelConstants.LL_NAME, LLFunnelConstants.CAMERA_OFFSET, true);
    private final ElevSubsystem elev = new ElevSubsystem();
    private final ClimbSubsystem climb = new ClimbSubsystem();
    private final HeadSubsystem head = new HeadSubsystem();
    private final IntakeSubsystem intake = new IntakeSubsystem();
    private final AlgaeRemSubsystem algaeRem = new AlgaeRemSubsystem();
    private final LEDSubsystem leds = new LEDSubsystem();
    
    private SendableChooser<Command> autoChooser;
    private TeleopDrive teleopDrive;
    private LEDManager ledManager;
    private LoggingManager logging;
    
    public RobotContainer() {
        logging = new LoggingManager(swerve, reeflimelight, funnellimelight, elev, climb);

        driver1 = new CommandXboxController(0);
        driver2 = new CommandXboxController(1);
        hid1 = driver1.getHID(); //use hid objects to reduce performance impact. Using getBoolean() on the trigger from CommandXboxController causes large CPU usage
        hid2 = driver2.getHID();

        teleopDrive = new TeleopDrive(swerve,
            () -> -MathUtil.applyDeadband(hid1.getLeftY(), SwerveConstants.LEFT_Y_DEADBAND),
            () -> -MathUtil.applyDeadband(hid1.getLeftX(), SwerveConstants.LEFT_X_DEADBAND),
            () -> -MathUtil.applyDeadband(hid1.getRightX(), SwerveConstants.RIGHT_X_DEADBAND),
            () -> hid1.getPOV(),
            () -> hid1.getLeftTriggerAxis() > SwerveConstants.TRIGGER_DEADBAND,
            () -> hid1.getBackButton(),
            () -> MathUtil.applyDeadband(hid1.getRightTriggerAxis(), SwerveConstants.TRIGGER_DEADBAND)
        );
        swerve.setDefaultCommand(teleopDrive);

        ledManager = new LEDManager(leds, head, elev, driver1, driver2);

        intake.setDefaultCommand(new Intake(intake, elev, head));
        leds.setDefaultCommand(ledManager);
        
        configureNamedCommands();
        configureBindings();
        addTelemetry();

        autoChooser = AutoBuilder.buildAutoChooser();
        setupAutoDisplay();
        SmartDashboard.putData("Auto Chooser", autoChooser);

        Field2d field = new Field2d();
        SmartDashboard.putData("Field", field);
    }

    private void configureBindings() {
        driver1.b().onTrue(Commands.runOnce(swerve::zeroGyroWithAlliance));
        driver1.x().whileTrue(Commands.runOnce(swerve::lock, swerve).repeatedly());

        // limelight testing
        // driver1.a().onTrue(Commands.runOnce(swerve::addFakeVision(Reading));

        driver1.a().whileTrue(new AlignToTag(swerve, reeflimelight, ReefPosition.RIGHT));
        
        driver1.rightBumper().onTrue(new DriveToTag(swerve, true,
                () -> (Math.abs(driver1.getLeftX()) < SwerveConstants.LEFT_X_DEADBAND
                        && Math.abs(driver1.getLeftY()) < SwerveConstants.LEFT_Y_DEADBAND
                        && Math.abs(driver1.getRightX()) < SwerveConstants.RIGHT_X_DEADBAND),
                ReefPosition.RIGHT)); // Drive to closest tag
        driver1.leftBumper().onTrue(new DriveToTag(swerve, true,
                () -> (Math.abs(driver1.getLeftX()) < SwerveConstants.LEFT_X_DEADBAND
                        && Math.abs(driver1.getLeftY()) < SwerveConstants.LEFT_Y_DEADBAND
                        && Math.abs(driver1.getRightX()) < SwerveConstants.RIGHT_X_DEADBAND),
                ReefPosition.LEFT));
        driver1.y().onTrue(new DriveToTag(swerve, false,
                () -> (Math.abs(driver1.getLeftX()) < SwerveConstants.LEFT_X_DEADBAND
                        && Math.abs(driver1.getLeftY()) < SwerveConstants.LEFT_Y_DEADBAND
                        && Math.abs(driver1.getRightX()) < SwerveConstants.RIGHT_X_DEADBAND),
                ReefPosition.MIDDLE));
        
        // driver1.a().whileTrue(new DriveToAlgae(swerve, reeflimelight));
        
        driver2.a().onTrue(new Climb(climb));
        driver2.y().onTrue(new Declimb(climb));
        // driver2.back().onTrue(new ClimbOverride(climb));
        driver2.povUp().onTrue(new MoveToLevel(elev, head, Level.LVL4));
        driver2.povRight().onTrue(new MoveToLevel(elev, head, Level.LVL3));
        driver2.povLeft().onTrue(new MoveToLevel(elev, head, Level.LVL2));
        driver2.povDown().onTrue(new MoveToLevel(elev, head, Level.LVL1));
        driver2.leftTrigger(SwerveConstants.TRIGGER_DEADBAND).onTrue(new MoveToLevel(elev, head, Level.HOME));
        driver2.x().whileTrue(new ManualIntake(intake));
        // driver2.rightTrigger(SwerveConstants.TRIGGER_DEADBAND).onTrue(new SequentialCommandGroup(
        //     new Shoot(head, elev),
        //     new WaitCommand(1.5),
        //     new MoveToLevel(elev, Level.HOME)
        // ));
        driver2.rightTrigger(SwerveConstants.TRIGGER_DEADBAND).onTrue(new Shoot(head, elev));
        driver2.rightBumper().whileTrue(new SequentialCommandGroup(
            new MoveToLevel(elev, head, Level.ALGAE_HIGH, true),
            new AlgaeRemSpin(algaeRem, false)
        ));
        driver2.leftBumper().whileTrue(new SequentialCommandGroup(
            new MoveToLevel(elev, head, Level.ALGAE_LOW, true),
            new AlgaeRemSpin(algaeRem, false)
        ));
    }
    private void configureNamedCommands() {
        //Pathplanner named commands

        /*
         * Please use consitent naming conventions!!
         * <Subsytem><Action>
         * Use PascalCase
         */
        NamedCommands.registerCommand("SwerveDriveToReefDist", new DriveToReefDist(swerve, head));
        NamedCommands.registerCommand("SwerveDriveToSourceDist", new DriveToSourceDist(swerve, head));
        NamedCommands.registerCommand("SwerveAlignToBranch", new AlignToBranch(swerve, head, elev));
        
        NamedCommands.registerCommand("ElevHome", new MoveToLevel(elev, head, ElevSubsystem.Level.HOME, true));
        NamedCommands.registerCommand("ElevLevel1", new MoveToLevel(elev, head, ElevSubsystem.Level.LVL1, true));
        NamedCommands.registerCommand("ElevLevel2", new MoveToLevel(elev, head, ElevSubsystem.Level.LVL2, true));
        NamedCommands.registerCommand("ElevLevel3", new MoveToLevel(elev, head, ElevSubsystem.Level.LVL3, true));
        NamedCommands.registerCommand("ElevLevel4", new MoveToLevel(elev, head, ElevSubsystem.Level.LVL4, true));
        NamedCommands.registerCommand("ElevAlgaeLow", new MoveToLevel(elev, head, ElevSubsystem.Level.ALGAE_LOW, true));
        NamedCommands.registerCommand("ElevAlgaeHigh", new MoveToLevel(elev, head, ElevSubsystem.Level.ALGAE_HIGH, true));

        NamedCommands.registerCommand("HeadShoot", new Shoot(head, elev));
        NamedCommands.registerCommand("HeadShootSlow", new ShootSlow(head, elev));
        NamedCommands.registerCommand("HeadWaitForCoral", new WaitForHeadCoral(head));
        NamedCommands.registerCommand("IntakeWaitForCoral", new WaitForIntakeCoral(head, intake));

        NamedCommands.registerCommand("AlgaeRemSpin", new AlgaeRemSpin(algaeRem, true));
    }
    private void addTelemetry() {
        //one time telemetry values, such as dashboard commands
        if(TelemetryConstants.debugTelemetry) {
            SmartDashboard.putData("Swerve DriveToReefDist", new DriveToReefDist(swerve, head));
            SmartDashboard.putData("Swerve DriveToSourceDist", new DriveToSourceDist(swerve, head));
            SmartDashboard.putData("Swerve AlignToBranch", new AlignToBranch(swerve, head, elev));

            SmartDashboard.putData("Elev Manual Move (IN)", new MoveToInchesManual(elev));
            SmartDashboard.putData("Elev Manual Move (TICKS)", new MoveToTicksManual(elev));
            SmartDashboard.putData("Elev Manual Move (LVL)", new MoveToLevelManual(elev));
            SmartDashboard.putData("Elev Zero", new InstantCommand(elev::setAsZero).ignoringDisable(true));

            SmartDashboard.putData("Climb Lever Manual (DEG)", new ClimbLeverDegreesManual(climb));
            SmartDashboard.putData("Climb Clamp Manual (DEG)", new ClimbClampDegreesManual(climb));
            SmartDashboard.putData("Climb Manual", new Climb(climb));
            SmartDashboard.putData("Declimb Manual", new Declimb(climb));
            SmartDashboard.putData("Climb Slow", new Climb(climb, ClimbConstants.leverSlowVbus));

            SmartDashboard.putData("Head Shoot", new Shoot(head, elev));
            SmartDashboard.putData("Head ShootSlow", new ShootSlow(head, elev));

            SmartDashboard.putData("AlgaeRem Spin", new AlgaeRemSpin(algaeRem, true));
        }
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
    public HeadSubsystem getHeadSubsystem() {
        return head;
    }
    public void setTeleopDefaultCommand() {
        swerve.setDefaultCommand(teleopDrive);
    }
    public void clearTeleopDefaultCommand() {
        swerve.setDefaultCommand(new SwerveZero(swerve));
    }
    public void scheduleLimelight() {
        new LimelightManager(swerve, reeflimelight, funnellimelight).schedule();
    }
    public void scheduleLimelightAuton() {
        new ParallelRaceGroup(
            new LimelightManager(swerve, reeflimelight, funnellimelight),
            new WaitCommand(SwerveConstants.autonVisionTime)
        ).schedule();
    }
    public void scheduleControllerRumble() {
        new IntakeRumble(head, driver1, driver2).schedule();
    }
    public void stopRumble() {
        driver1.setRumble(RumbleType.kBothRumble, 0);
        driver2.setRumble(RumbleType.kBothRumble, 0);
    }
    public LEDManager getLEDManager() {
        return ledManager;
    }
    public void scheduleLoggingManager() {
        if(!logging.isScheduled())
            logging.schedule();
    }
    public void cancelLoggingManager() {
        if(logging.isScheduled())
            logging.cancel();
    }
}