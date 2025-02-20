package frc.robot.subsystems;

import com.revrobotics.servohub.ServoChannel;
import com.revrobotics.servohub.ServoChannel.ChannelId;
import com.revrobotics.servohub.ServoHub;
import com.revrobotics.servohub.config.ServoChannelConfig;
import com.revrobotics.servohub.config.ServoChannelConfig.BehaviorWhenDisabled;
import com.revrobotics.servohub.config.ServoHubConfig;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.AlgaeRemConstants;

public class AlgaeRemSubsystem extends SubsystemBase {
    
    private ServoHub hub;
    private ServoChannel servo1;
    private ServoChannel servo2;
    private SparkMax flywheel;

    public AlgaeRemSubsystem() {
        hub = new ServoHub(AlgaeRemConstants.servoHubId);

        configureHub();

        servo1 = hub.getServoChannel(ChannelId.fromInt(AlgaeRemConstants.servo1Id));
        servo1.setEnabled(true);
        servo1.setPowered(true);

        servo2 = hub.getServoChannel(ChannelId.fromInt(AlgaeRemConstants.servo2Id));
        servo2.setEnabled(true);
        servo2.setPowered(true);

        flywheel = new SparkMax(AlgaeRemConstants.flywheelId, MotorType.kBrushless);
        configureMotor(flywheel);

        flywheel.set(0);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("AlgaeRem Setpoint (Pulse)", getSetpointPulse());
        SmartDashboard.putNumber("AlgaeRem Setpoint 1 (Pulse)", servo1.getPulseWidth());
        SmartDashboard.putNumber("AlgaeRem Setpoint 2 (Pulse)", servo2.getPulseWidth());
        SmartDashboard.putNumber("AlgaeRem Setpoint (Perc)", getSetpointPercent());
        SmartDashboard.putNumber("AlgaeRem Setpoint 1 (Perc)", getSetpointPercent1());
        SmartDashboard.putNumber("AlgaeRem Setpoint 2 (Perc)", getSetpointPercent2());
        SmartDashboard.putNumber("AlgaeRem VBus", flywheel.get());
    }

    public void goToPercent(double percent) {
        percent = MathUtil.clamp(percent, 0, 1);

        int pulse = (int) (percent * (AlgaeRemConstants.servoMax - AlgaeRemConstants.servoMin) + AlgaeRemConstants.servoMin);
        int pulseReversed = AlgaeRemConstants.servoMax - pulse;

        servo1.setPulseWidth(AlgaeRemConstants.servo1Reversed ? pulseReversed : pulse);
        servo2.setPulseWidth(AlgaeRemConstants.servo2Reversed ? pulseReversed : pulse);
    }
    public void setFlywheelVBus(double percent) {
        flywheel.set(percent);
    }
    
    public double getSetpointPercent() {
        double servo1Perc = getSetpointPercent1();
        double servo2Perc = getSetpointPercent2();

        if(Math.abs(servo1Perc - servo2Perc) > 0.0001 /*Allow floating point error*/) {
            DriverStation.reportWarning("AlgaeRemoverSubsystem: servo1 and servo2 don't have the same setpoint", false);
            
            //use midpoint
            return (servo1Perc + servo2Perc) / 2;
        }

        //use either servo, doesn't matter
        return servo1Perc;
    }

    private void configureHub() {
        ServoHubConfig config = new ServoHubConfig();
        
        int min = AlgaeRemConstants.servoMin;
        int center = (AlgaeRemConstants.servoMin + AlgaeRemConstants.servoMax) / 2;
        int max = AlgaeRemConstants.servoMax;

        ServoChannelConfig servo1Config = new ServoChannelConfig(ChannelId.fromInt(AlgaeRemConstants.servo1Id));
        servo1Config.disableBehavior(BehaviorWhenDisabled.kSupplyPower);
        servo1Config.pulseRange(min, center, max);

        ServoChannelConfig servo2Config = new ServoChannelConfig(ChannelId.fromInt(AlgaeRemConstants.servo2Id));
        servo2Config.disableBehavior(BehaviorWhenDisabled.kSupplyPower);
        servo2Config.pulseRange(min, center, max);
        
        config.apply(ChannelId.fromInt(AlgaeRemConstants.servo1Id), servo1Config);
        config.apply(ChannelId.fromInt(AlgaeRemConstants.servo2Id), servo2Config);
        hub.configure(config, ServoHub.ResetMode.kResetSafeParameters);
    }
    private void configureMotor(SparkMax m) {
        SparkMaxConfig config = new SparkMaxConfig();
        config.idleMode(IdleMode.kCoast);
        config.smartCurrentLimit(AlgaeRemConstants.flywheelCurrentLimit);

        m.configure(config, SparkMax.ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
    private double getSetpointPercent1() {
        return (double) (servo1.getPulseWidth() - AlgaeRemConstants.servoMin) / (AlgaeRemConstants.servoMax - AlgaeRemConstants.servoMin);
    }
    private double getSetpointPercent2() {
        return (double) (servo2.getPulseWidth() - AlgaeRemConstants.servoMin) / (AlgaeRemConstants.servoMax - AlgaeRemConstants.servoMin);
    }
    private int getSetpointPulse() {
        int servo1Pulse = servo1.getPulseWidth();
        int servo2Pulse = servo2.getPulseWidth();

        if(servo1Pulse != servo2Pulse) {
            DriverStation.reportWarning("AlgaeRemoverSubsystem: servo1 and servo2 don't have the same setpoint", false);
            
            //use midpoint
            return (servo1Pulse + servo2Pulse) / 2;
        }

        //use either servo, doesn't matter
        return servo1Pulse;
    }
}
