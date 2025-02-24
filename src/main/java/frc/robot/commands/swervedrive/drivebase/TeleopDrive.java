package frc.robot.commands.swervedrive.drivebase;

import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.SwerveConstants;
import frc.robot.constants.VisionConstants.ApriltagConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class TeleopDrive extends Command {

    private final SwerveSubsystem swerve;
    private final DoubleSupplier vX, vY;
    private final DoubleSupplier heading;
    private final BooleanSupplier povCenter, povDown, povDownleft, povDownRight, povLeft, povRight, povUp, povUpLeft,
            povUpRight;
    private final BooleanSupplier reefLock;
    private final BooleanSupplier centricToggle;
    private final BooleanSupplier shiftHalf, shiftQuarter;

    private boolean isFieldCentric = true;
    private boolean centricToggleLast = false;
    private boolean usingPOV = false;

    public TeleopDrive(SwerveSubsystem swerve, DoubleSupplier vX, DoubleSupplier vY, DoubleSupplier heading,
            BooleanSupplier povCenter, BooleanSupplier povDown, BooleanSupplier povDownleft,
            BooleanSupplier povDownRight, BooleanSupplier povLeft, BooleanSupplier povRight, BooleanSupplier povUp,
            BooleanSupplier povUpLeft, BooleanSupplier povUpRight, BooleanSupplier reefLock,
            BooleanSupplier centricToggle, BooleanSupplier shiftHalf, BooleanSupplier shiftQuarter) {
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
        this.shiftHalf = shiftHalf;
        this.shiftQuarter = shiftQuarter;
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        // Handle field-centric toggle
        if (centricToggle.getAsBoolean() && !centricToggleLast) {
            isFieldCentric = !isFieldCentric;
        }
        centricToggleLast = centricToggle.getAsBoolean();

        if(heading.getAsDouble() != 0) {
            usingPOV = false;
        }
        else if(!povCenter.getAsBoolean()) {
            usingPOV = true;
        }

        // Calculate speed multiplier
        double shiftScalar = 1;
        if (shiftQuarter.getAsBoolean())
            shiftScalar = 0.25;
        else if (shiftHalf.getAsBoolean())
            shiftScalar = 0.5;

        // Calculate rotation
        double rotationSpeed = heading.getAsDouble() * swerve.getSwerveController().config.maxAngularVelocity;
        rotationSpeed *= shiftScalar * SwerveConstants.DRIVE_SPEED;

        // Calculate translation
        Translation2d translation = new Translation2d(vX.getAsDouble(), vY.getAsDouble())
                .times(SwerveConstants.MAX_SPEED)
                .times(shiftScalar)
                .times(SwerveConstants.DRIVE_SPEED)
                .times(isFieldCentric ? swerve.isRedAlliance() ? -1 : 1 : 1);

        if (usingPOV) {
            // Drive with POV
            swerve.drive(translation, POVDrive(), isFieldCentric);
        } else if (reefLock.getAsBoolean()) {
            // Drive with Reef Rotation
            swerve.drive(translation, ReefLock(), isFieldCentric);
        } else {
            // Drive normally
            swerve.drive(translation, rotationSpeed, isFieldCentric);
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
        List<Pose2d> tagposes = new ArrayList<>();
        for (int i = 1; i <= 22; i++) {
            if (i == 4 || i == 5 || i == 14 || i == 15 || i == 3 || i == 16)
                continue;
            tagposes.add(ApriltagConstants.TAG_POSES[i].toPose2d());
        }

        Pose2d nearestTag = swerve.getPose().nearest(tagposes);
        Rotation2d targetRotation = nearestTag.getRotation().minus(new Rotation2d(Math.PI));

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