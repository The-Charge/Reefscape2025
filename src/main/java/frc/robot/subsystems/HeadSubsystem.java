package frc.robot.subsystems;

import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.HeadConstants;
import frc.robot.constants.TelemetryConstants;

public class HeadSubsystem extends SubsystemBase {

    private TimeOfFlight funnelSensor;
    private TimeOfFlight shooterSensor;
    private SparkMax headLeft;
    private SparkMax headRight;
    private boolean hasCoral = false;
    
    public HeadSubsystem() {
        funnelSensor = new TimeOfFlight(HeadConstants.funnelSensorId);
        shooterSensor = new TimeOfFlight(HeadConstants.shooterSensorId);

        funnelSensor.setRangingMode(RangingMode.Short, HeadConstants.sensorSampleTime);
        shooterSensor.setRangingMode(RangingMode.Short, HeadConstants.sensorSampleTime);

        headLeft = new SparkMax(HeadConstants.leftId, MotorType.kBrushless);
        headRight = new SparkMax(HeadConstants.rightId, MotorType.kBrushless);

        configureMotor(headLeft, HeadConstants.leftReversed);
        configureMotor(headRight, HeadConstants.rightReversed);

        headLeft.set(0);
        headRight.set(0);

        createSensorTriggers();
    }

    @Override
    public void periodic() {
        if(TelemetryConstants.headLevel >= TelemetryConstants.LOW) {
            SmartDashboard.putNumber("Head VBus L", headLeft.get());
            SmartDashboard.putNumber("Head VBus R", headRight.get());
            SmartDashboard.putBoolean("Head HasCoral", getHasCoral());
            
            if(TelemetryConstants.headLevel >= TelemetryConstants.MEDIUM) {
                SmartDashboard.putBoolean("Head Funnel Sensor (Bool)", getFunnelSensor());
                SmartDashboard.putBoolean("Head Shooter Sensor (Bool)", getShooterSensor());

                if(TelemetryConstants.headLevel >= TelemetryConstants.HIGH) {
                    SmartDashboard.putNumber("Head Funnel Sensor (mm)", funnelSensor.getRange());
                    SmartDashboard.putNumber("Head Shooter Sensor (mm)", shooterSensor.getRange());
    
                    if(TelemetryConstants.headLevel >= TelemetryConstants.EYE_OF_SAURON) {
                        SmartDashboard.putNumber("Head Current L", headLeft.getOutputCurrent());
                        SmartDashboard.putNumber("Head Current R", headRight.getOutputCurrent());
                        if(getCurrentCommand() == null)
                            SmartDashboard.putString("Head RunningCommand", "None");
                        else
                            SmartDashboard.putString("Head RunningCommand", getCurrentCommand().getName());
                    }
                }
            }
        }
    }

    public void flywheelVBus(double percent) {
        headLeft.set(percent);
        headRight.set(percent);
    }
    public void stop() {
        headLeft.set(0);
        headRight.set(0);
    }

    public boolean getFunnelSensor() {
        return funnelSensor.getRange() <= HeadConstants.sensorActivationDist;
    }
    public boolean getShooterSensor() {
        return shooterSensor.getRange() <= HeadConstants.sensorActivationDist;
    }
    public boolean getHasCoral() {
        return hasCoral;
    }

    private void configureMotor(SparkMax m, boolean inverted) {
        SparkMaxConfig config = new SparkMaxConfig();
        config.idleMode(HeadConstants.idleMode);
        config.smartCurrentLimit(HeadConstants.currentLimit);
        config.inverted(inverted);

        m.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
    private void createSensorTriggers() {
        new Trigger(() -> getShooterSensor()).onTrue(new InstantCommand(() -> {
            hasCoral = true;
        }));
        new Trigger(() -> getShooterSensor()).onFalse(new SequentialCommandGroup(
            new WaitCommand(HeadConstants.hasCoralDeactivationDelay),
            new InstantCommand(() -> {
                hasCoral = false;
            })
        ));
    }
}
