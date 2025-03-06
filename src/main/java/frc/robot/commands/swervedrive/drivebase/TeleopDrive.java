package frc.robot.commands.swervedrive.drivebase;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.SwerveConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class TeleopDrive extends Command {

    private final SwerveSubsystem swerve;
    private final DoubleSupplier vX, vY;
    private final DoubleSupplier heading;
    private final BooleanSupplier povCenter, povDown, povDownleft, povDownRight, povLeft, povRight, povUp, povUpLeft,
            povUpRight;
    private final DoubleSupplier shiftScalar;
    private final BooleanSupplier reefLock;
    private final BooleanSupplier centricToggle;

    private boolean isFieldCentric = true;
    private boolean centricToggleLast = false;

    private enum Mode {JOYSTICK, POV, REEF};

    private Mode mode = Mode.JOYSTICK;

    public TeleopDrive(SwerveSubsystem swerve, DoubleSupplier vX, DoubleSupplier vY, DoubleSupplier heading,
            BooleanSupplier povCenter, BooleanSupplier povDown, BooleanSupplier povDownleft,
            BooleanSupplier povDownRight, BooleanSupplier povLeft, BooleanSupplier povRight, BooleanSupplier povUp,
            BooleanSupplier povUpLeft, BooleanSupplier povUpRight, BooleanSupplier reefLock,
            BooleanSupplier centricToggle, DoubleSupplier shift) {
        this.swerve = swerve;
        this.vX = vX;
        this.vY = vY;
        this.heading = heading;
        this.povCenter = povCenter;
        this.povDown = povDown;
        this.povDownleft = povDownleft;
        this.povDownRight = povDownRight;
        this.povLeft = povLeft;
        this.povRight = povRight;
        this.povUp = povUp;
        this.povUpLeft = povUpLeft;
        this.povUpRight = povUpRight;
        this.reefLock = reefLock;
        this.centricToggle = centricToggle;
        this.shiftScalar = shift;
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        SmartDashboard.putBoolean("Swerve IsFieldCentric", isFieldCentric);
    }

    @Override
    public void execute() {
        // Handle field-centric toggle
        if (centricToggle.getAsBoolean() && !centricToggleLast) {
            isFieldCentric = !isFieldCentric;
            SmartDashboard.putBoolean("Swerve IsFieldCentric", isFieldCentric);
        }
        centricToggleLast = centricToggle.getAsBoolean();

        // reefLock get priority, then POV, then normal
        if (reefLock.getAsBoolean()) {
            mode = Mode.REEF;
        } else if(!povCenter.getAsBoolean()) {
            mode = Mode.POV;
        } else if (Math.abs(heading.getAsDouble()) >= SwerveConstants.RIGHT_X_DEADBAND) {
            mode = Mode.JOYSTICK;
        }

        // Calculate speed multiplier
        double shiftAmt = -0.9 * shiftScalar.getAsDouble() + 1;

        // Calculate rotation
        double rotationSpeed = heading.getAsDouble() * swerve.getSwerveController().config.maxAngularVelocity;
        rotationSpeed *= shiftAmt * SwerveConstants.DRIVE_SPEED;

        // Calculate translation
        Translation2d translation = new Translation2d(vX.getAsDouble(), vY.getAsDouble())
                .times(SwerveConstants.MAX_SPEED)
                .times(shiftAmt)
                .times(SwerveConstants.DRIVE_SPEED)
                .times(isFieldCentric ? swerve.isRedAlliance() ? -1 : 1 : 1);

        switch (mode) {
        case POV:
            // Drive with POV
            swerve.drive(translation, POVDrive(), isFieldCentric);
            break;
        case REEF:
            // Drive with Reef Rotation
            swerve.drive(translation, ReefLock(), isFieldCentric);
            break;
        default:
            // Drive normally
            swerve.drive(translation, rotationSpeed, isFieldCentric);
            break;
        }
    }

    private double POVDrive() {
        double headingX = 0;
        double headingY = 0;

        if (povDown.getAsBoolean()) {
            headingY = -1;
        } else if (povDownleft.getAsBoolean()) {
            headingX = 1;
            headingY = -1;
        } else if (povDownRight.getAsBoolean()) {
            headingX = -1;
            headingY = -1;
        } else if (povLeft.getAsBoolean()) {
            headingX = 1;
        } else if (povRight.getAsBoolean()) {
            headingX = -1;
        } else if (povUp.getAsBoolean()) {
            headingY = 1;
        } else if (povUpLeft.getAsBoolean()) {
            headingX = 1;
            headingY = 1;
        } else if (povUpRight.getAsBoolean()) {
            headingX = -1;
            headingY = 1;
        }

        double isRedAlliance = swerve.isRedAlliance() ? -1 : 1;
        ChassisSpeeds povSpeeds = swerve.getTargetSpeeds(vY.getAsDouble(), vX.getAsDouble(), headingX * isRedAlliance,
                headingY * isRedAlliance);

        return povSpeeds.omegaRadiansPerSecond;
    }
    
    private double ReefLock() {
        int tag = swerve.getClosestTagID();
        Rotation2d targetRotation = swerve.getClosestTagPose().getRotation();
        if (!(tag == 1 || tag == 2 || tag == 12 || tag == 13)) {
            targetRotation = targetRotation.minus(Rotation2d.fromDegrees(180));
        }
        ChassisSpeeds reefLockSpeeds = swerve.getTargetSpeeds(
                vX.getAsDouble(), vY.getAsDouble(),
                targetRotation.getSin(),
                targetRotation.getCos());
        return reefLockSpeeds.omegaRadiansPerSecond;
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}