package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.AlgaeRemConstants;

public class AlgaeRemSubsystem extends SubsystemBase {
    
    private SparkMax flywheel;

    public AlgaeRemSubsystem() {
        flywheel = new SparkMax(AlgaeRemConstants.flywheelId, MotorType.kBrushless);
        configureMotor(flywheel);

        flywheel.set(0);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("AlgaeRem VBus", flywheel.get());
    }

    public void setFlywheelVBus(double percent) {
        flywheel.set(percent);
    }
    public void stop() {
        flywheel.set(0);
    }
    
    private void configureMotor(SparkMax m) {
        SparkMaxConfig config = new SparkMaxConfig();
        config.idleMode(IdleMode.kCoast);
        config.smartCurrentLimit(AlgaeRemConstants.flywheelCurrentLimit);

        m.configure(config, SparkMax.ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
}
