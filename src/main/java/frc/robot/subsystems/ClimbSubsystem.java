package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ClimbConstants;

public class ClimbSubsystem extends SubsystemBase {
    
    public enum State {
        ACTIVE,
        RESTING,
        UNKNOWN
    }

    private TalonFX motor;
    private double targetTicks = 0;
    private int targetCounter = 0;
    private boolean isAtTarget = true;

    public ClimbSubsystem() {
        motor = new TalonFX(ClimbConstants.motorID);

        configureMotor(motor);

        //used for smartdashboard override commands, read only
        SmartDashboard.putNumber(ClimbConstants.overrideDegName, 0);
        SmartDashboard.putNumber(ClimbConstants.overrideTicksName, 0);
    }

    @Override
    public void periodic() {
        targetCheck();

        SmartDashboard.putNumber("Climb Ang (Deg)", getAngleDegrees());
        SmartDashboard.putNumber("Climb Ang (Ticks)", getAngleTicks());
        SmartDashboard.putString("Climb Ang (STA)", getAngleState().name());
        SmartDashboard.putNumber("Climb Err (Deg)", (targetTicks - getAngleTicks()) * ClimbConstants.tickToDegConversion);
        SmartDashboard.putNumber("Climb Err (Ticks)", targetTicks - getAngleTicks());
        SmartDashboard.putNumber("Climb Target (Deg)", targetTicks * ClimbConstants.tickToDegConversion);
        SmartDashboard.putNumber("Climb Target (Ticks)", targetTicks);
        SmartDashboard.putString("Climb Target (STA)", getTargetState().name());
        SmartDashboard.putBoolean("Climb isAtTarget", isAtTarget());
    }

    public void setTargetAngleDegrees(double deg) {
        setTargetAngleTicks(deg / ClimbConstants.tickToDegConversion);
    }
    public void setTargetAngleTicks(double ticks) {
        targetTicks = MathUtil.clamp(ticks, ClimbConstants.minPosTicks, ClimbConstants.maxPosTicks);
        
        PositionDutyCycle request = new PositionDutyCycle(targetTicks).withSlot(0);
        motor.setControl(request);

        resetTargetCounter();
    }
    public void setTargetPositionLevel(State state) {
        double val;

        switch(state) {
            case ACTIVE:
            val = ClimbConstants.activeDegrees;
            break;

            case RESTING:
            val = ClimbConstants.restingDegrees;
            break;

            case UNKNOWN:
            default:
            DriverStation.reportWarning("Cannot rotate climb to UNKNOWN level", false);
            return;
        }

        setTargetAngleDegrees(val);
    }
    public void stop() {
        motor.set(0);
    }

    public double getAngleDegrees() {
        return getAngleTicks() * ClimbConstants.tickToDegConversion;
    }
    public double getAngleTicks() {
        return motor.getPosition().getValueAsDouble();
    }
    public State getAngleState() {
        return getCurrentState(getAngleDegrees());
    }
    public boolean isAtTarget() {
        return isAtTarget;
    }

    private void configureMotor(TalonFX m) {
        TalonFXConfiguration motorConfig = new TalonFXConfiguration();
        motorConfig.MotorOutput.PeakForwardDutyCycle = ClimbConstants.maxVBus;
        motorConfig.MotorOutput.PeakReverseDutyCycle = -ClimbConstants.maxVBus;
        motorConfig.MotorOutput.withNeutralMode(ClimbConstants.neutralMode);
        motorConfig.MotorOutput.Inverted = ClimbConstants.inverted;
        
        motorConfig.CurrentLimits.StatorCurrentLimit = ClimbConstants.maxCurrent;
        motorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        
        Slot0Configs slotConfigs = motorConfig.Slot0;
        slotConfigs.kS = slotConfigs.kV = 0;
        slotConfigs.kP = ClimbConstants.pidf.p;
        slotConfigs.kI = ClimbConstants.pidf.i;
        slotConfigs.kD = ClimbConstants.pidf.d;
        slotConfigs.kG = ClimbConstants.pidf.f;
        slotConfigs.GravityType = GravityTypeValue.Elevator_Static;
        
        m.getConfigurator().apply(motorConfig);
        
        SoftwareLimitSwitchConfigs softLimits = new SoftwareLimitSwitchConfigs();
        softLimits.ForwardSoftLimitEnable = softLimits.ReverseSoftLimitEnable = true;
        softLimits.ForwardSoftLimitThreshold = ClimbConstants.maxPosTicks;
        softLimits.ReverseSoftLimitThreshold = ClimbConstants.minPosTicks;
        m.getConfigurator().apply(softLimits);
    }
    private void resetTargetCounter() {
        targetCounter = 0;
        isAtTarget = false; //to prevent a single frame where the target has been changed but the boolean hasnt been updated
    }
    private State getCurrentState(double deg) {
        if(Math.abs(deg - ClimbConstants.activeDegrees) <= ClimbConstants.targetThresholdDegrees)
            return State.ACTIVE;
        else if(Math.abs(deg - ClimbConstants.restingDegrees) <= ClimbConstants.targetThresholdDegrees)
            return State.RESTING;
        
        return State.UNKNOWN;
    }
    private State getTargetState() {
        return getCurrentState(targetTicks * ClimbConstants.tickToDegConversion);
    }
    /**
     * ONLY RUN ONCE PER UPDATE!!
     */
    private void targetCheck() {
        double err = Math.abs(targetTicks - getAngleTicks());
        if(err <= ClimbConstants.targetThresholdDegrees / ClimbConstants.tickToDegConversion)
            targetCounter++;
        else
            resetTargetCounter();
        
        if(targetCounter >= ClimbConstants.targetThresholdSeconds * 50)
            isAtTarget = true;
        else
            isAtTarget = false;
    }
}
