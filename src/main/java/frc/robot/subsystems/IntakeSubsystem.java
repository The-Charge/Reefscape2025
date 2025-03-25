package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.HeadConstants;
import frc.robot.constants.IntakeConstants;
import frc.robot.constants.TelemetryConstants;

public class IntakeSubsystem extends SubsystemBase {
    
    private SparkMax belt;
    
    public IntakeSubsystem() {
        belt = new SparkMax(IntakeConstants.beltID, MotorType.kBrushless);

        configureMotor(belt);

        belt.set(0);
    }

    @Override
    public void periodic() {
        if(getCurrentCommand() == null)
            SmartDashboard.putString("Intake RunningCommand", "None");
        else
            SmartDashboard.putString("Intake RunningCommand", getCurrentCommand().getName());

        if(TelemetryConstants.debugTelemetry) {
            SmartDashboard.putNumber("Intake VBus", belt.get());
            SmartDashboard.putNumber("Intake Velocity", belt.getEncoder().getVelocity());

            SmartDashboard.putNumber("Intake Current", belt.getOutputCurrent());
        }
    }

    public void vBus(double percent) {
        belt.set(percent);
    }
    public void stop() {
        belt.set(0);
    }
    
    public double getVelocity() {
        return belt.getEncoder().getVelocity();
    }

    public double getAmps() {
        return belt.getOutputCurrent();
    }

    private void configureMotor(SparkMax m) {
        SparkMaxConfig config = new SparkMaxConfig();
        config.idleMode(IdleMode.kCoast);
        config.smartCurrentLimit(HeadConstants.currentLimit);
        config.inverted(IntakeConstants.inverted);

        m.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
}
