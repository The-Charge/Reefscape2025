package frc.robot.subsystems;

import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.LoggingManager;
import frc.robot.constants.HeadConstants;
import frc.robot.constants.TelemetryConstants;

public class HeadSubsystem extends SubsystemBase {

    private TimeOfFlight branchSensor;
    private TimeOfFlight coralSensor;
    private TimeOfFlight backSensor;
    private TimeOfFlight frontSensor;
    private SparkMax headLeft;
    private SparkMax headRight;
    private boolean hasCoral = false;
    
    public HeadSubsystem() {
        branchSensor = new TimeOfFlight(HeadConstants.branchSensorId);
        coralSensor = new TimeOfFlight(HeadConstants.coralSensorId);
        backSensor = new TimeOfFlight(HeadConstants.backSensorId);
        frontSensor = new TimeOfFlight(HeadConstants.frontSensorId);

        branchSensor.setRangingMode(RangingMode.Long, HeadConstants.longSensorSampleTime);
        coralSensor.setRangingMode(RangingMode.Short, HeadConstants.shortSensorSampleTime);
        backSensor.setRangingMode(RangingMode.Long, HeadConstants.longSensorSampleTime);
        frontSensor.setRangingMode(RangingMode.Long, HeadConstants.longSensorSampleTime);
        branchSensor.setRangeOfInterest(8, 8, 12, 12);

        headLeft = new SparkMax(HeadConstants.leftId, MotorType.kBrushless);
        headRight = new SparkMax(HeadConstants.rightId, MotorType.kBrushless);

        configureMotor(headLeft, HeadConstants.leftReversed);
        configureMotor(headRight, HeadConstants.rightReversed);

        headLeft.set(0);
        headRight.set(0);
    }

    @Override
    public void periodic() {
        hasCoral = getCoralSensor();

        if(getCurrentCommand() == null)
            LoggingManager.logAndAutoSendValue("Head RunningCommand", "None");
        else
            LoggingManager.logAndAutoSendValue("Head RunningCommand", getCurrentCommand().getName());
        
        LoggingManager.logAndAutoSendValue("Head HasCoral", getHasCoral());
        LoggingManager.logAndAutoSendValue("Head Branch Sensor (mm)", branchSensor.getRange());
        LoggingManager.logAndAutoSendValue("Head Branch Sensor (sigma)", branchSensor.getRangeSigma());
        LoggingManager.logAndAutoSendValue("Head Back Sensor (mm)", backSensor.getRange());
        LoggingManager.logAndAutoSendValue("Head Front Sensor (mm)", frontSensor.getRange());
        LoggingManager.logAndAutoSendValue("Head Branch Sensor LVL2 (Bool)", getBranchSensor(ElevSubsystem.Level.LVL2));
        LoggingManager.logAndAutoSendValue("Head Branch Sensor LVL3 (Bool)", getBranchSensor(ElevSubsystem.Level.LVL3));
        LoggingManager.logAndAutoSendValue("Head Branch Sensor LVL4 (Bool)", getBranchSensor(ElevSubsystem.Level.LVL4));

        if(TelemetryConstants.debugTelemetry) {
            SmartDashboard.putNumber("Head VBus L", headLeft.get());
            SmartDashboard.putNumber("Head VBus R", headRight.get());
            
            SmartDashboard.putBoolean("Head Coral Sensor (Bool)", getCoralSensor());

            SmartDashboard.putNumber("Head Coral Sensor (mm)", coralSensor.getRange());
    
            SmartDashboard.putNumber("Head Current L", headLeft.getOutputCurrent());
            SmartDashboard.putNumber("Head Current R", headRight.getOutputCurrent());
        }
    }

    public void flywheelLeft(double percent) {
        headLeft.set(percent);
    }
    public void flywheelRight(double percent) {
        headRight.set(percent);
    }
    public void stop() {
        headLeft.set(0);
        headRight.set(0);
    }

    public boolean getBranchSensor(ElevSubsystem.Level level) {
        switch (level) {
            case LVL4:
                return branchSensor.getRange() <= HeadConstants.branchLVL4ActivationDist
                        && branchSensor.getRangeSigma() < HeadConstants.branchSigmaActivation;
            case LVL3:
                return branchSensor.getRange() <= HeadConstants.branchLVL3ActivationDist && branchSensor
                        .getRange() >= HeadConstants.branchLVL3MinActivationDist 
                        && branchSensor.getRangeSigma() < HeadConstants.branchSigmaActivation;
            case LVL2:
                return branchSensor.getRange() <= HeadConstants.branchLVL2ActivationDist 
                        && branchSensor.getRangeSigma() < HeadConstants.branchSigmaActivation;
            default:
                return true;
        }
    }

    public boolean getCoralSensor() {
        return coralSensor.getRange() <= HeadConstants.coralActivationDist;
    }
    public double getBackDistance() {
        return backSensor.getRange();
    }
    public double getFrontDistance() {
        return frontSensor.getRange();
    }
    public boolean getHasCoral() {
        return hasCoral;
    }

    public void recheckHasCoral() {
        hasCoral = getCoralSensor();
    }

    private void configureMotor(SparkMax m, boolean inverted) {
        SparkMaxConfig config = new SparkMaxConfig();
        config.idleMode(HeadConstants.idleMode);
        config.smartCurrentLimit(HeadConstants.currentLimit);
        config.inverted(inverted);

        m.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
}
