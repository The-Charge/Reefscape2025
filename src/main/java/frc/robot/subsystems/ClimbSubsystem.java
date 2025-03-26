package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ForwardLimitValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ClimbConstants;
import frc.robot.constants.TelemetryConstants;
import swervelib.parser.PIDFConfig;

public class ClimbSubsystem extends SubsystemBase {
    
    public enum State {
        ACTIVE,
        RESTING,
        UNKNOWN
    }

    private TalonFX lever, clamp;
    private double leverTargetTicks = 0;
    private double clampTargetTicks = 0;
    private int leverTargetCounter = 0;
    private int clampTargetCounter = 0;
    private boolean leverIsAtTarget = true;
    private boolean clampIsAtTarget = true;

    public ClimbSubsystem() {
        lever = new TalonFX(ClimbConstants.leverID);
        clamp = new TalonFX(ClimbConstants.clampID);

        configureMotor(lever, ClimbConstants.leverMaxVBus, ClimbConstants.leverNeutralMode, ClimbConstants.leverInverted, ClimbConstants.leverMaxCurrent, ClimbConstants.leverPIDF, ClimbConstants.leverMinPosTicks, ClimbConstants.leverMaxPosTicks, ClimbConstants.leverUseSoftLimits);
        configureMotor(clamp, ClimbConstants.clampMaxVBus, ClimbConstants.clampNeutralMode, ClimbConstants.clampInverted, ClimbConstants.clampMaxCurrent, ClimbConstants.clampPIDF, ClimbConstants.clampMinPosTicks, ClimbConstants.clampMaxPosTicks, ClimbConstants.clampUseSoftLimits);

        if(TelemetryConstants.debugTelemetry) {
            //used for smartdashboard override commands, read only
            SmartDashboard.putNumber(ClimbConstants.leverOverrideDegName, 0);
        }
    }

    @Override
    public void periodic() {
        leverTargetCheck();
        clampTargetCheck();

        if(getCurrentCommand() == null)
            SmartDashboard.putString("Climb RunningCommand", "None");
        else
            SmartDashboard.putString("Climb RunningCommand", getCurrentCommand().getName());
        
        SmartDashboard.putNumber("Climb Lever Ang (Ticks)", getLeverTicks());
        SmartDashboard.putNumber("Climb Lever Ang (Deg)", getLeverDegrees());
        SmartDashboard.putNumber("Climb Lever VBus", lever.get());
        SmartDashboard.putNumber("Climb Lever Current", lever.getStatorCurrent().getValueAsDouble());

        if(TelemetryConstants.debugTelemetry) {
            SmartDashboard.putString("Climb Lever Ang (STA)", getLeverState().name());
            SmartDashboard.putString("Climb Lever Target (STA)", getLeverTargetState().name());
            SmartDashboard.putBoolean("Climb Lever isAtTarget", isLeverIsAtTarget());
            SmartDashboard.putString("Climb Clamp Ang (STA)", getClampState().name());
            SmartDashboard.putString("Climb Clamp Target (STA)", getClampTargetState().name());
            SmartDashboard.putBoolean("Climb Clamp isAtTarget", isClampIsAtTarget());

            SmartDashboard.putNumber("Climb Lever Err (Deg)", (leverTargetTicks - getLeverTicks()) * ClimbConstants.leverTickToDegConversion);
            SmartDashboard.putNumber("Climb Lever Target (Deg)", leverTargetTicks * ClimbConstants.leverTickToDegConversion);
            SmartDashboard.putNumber("Climb Clamp Ang (Deg)", getClampDegrees());
            SmartDashboard.putNumber("Climb Clamp Err (Deg)", (clampTargetTicks - getClampTicks()) * ClimbConstants.clampTickToDegConversion);
            SmartDashboard.putNumber("Climb Clamp Target (Deg)", clampTargetTicks * ClimbConstants.clampTickToDegConversion);

            SmartDashboard.putNumber("Climb Lever Err (Ticks)", leverTargetTicks - getLeverTicks());
            SmartDashboard.putNumber("Climb Lever Target (Ticks)", leverTargetTicks);
            SmartDashboard.putNumber("Climb Clamp Ang (Ticks)", getClampTicks());
            SmartDashboard.putNumber("Climb Clamp Err (Ticks)", clampTargetTicks - getClampTicks());
            SmartDashboard.putNumber("Climb Clamp Target (Ticks)", clampTargetTicks);
            SmartDashboard.putBoolean("Climb Lever Swich", getLeverLimitSwitch());

            SmartDashboard.putNumber("Climb Lever Temp (deg C)", lever.getDeviceTemp().getValueAsDouble());
            SmartDashboard.putNumber("Climb Clamp VBus", clamp.get());
            SmartDashboard.putNumber("Climb Clamp Current", clamp.getStatorCurrent().getValueAsDouble());
            SmartDashboard.putNumber("Climb Clamp Temp (deg C)", clamp.getDeviceTemp().getValueAsDouble());
        }
    }

    public void setLeverDegrees(double deg) {
        setLeverTicks(deg / ClimbConstants.leverTickToDegConversion);
    }
    public void setLeverTicks(double ticks) {
        // if(getAngleDegrees() >= ClimbConstants.safeDegrees) {
        //     return;
        // }

        if(ClimbConstants.leverUseSoftLimits)
            leverTargetTicks = MathUtil.clamp(ticks, ClimbConstants.leverMinPosTicks, ClimbConstants.leverMaxPosTicks);
        else
            leverTargetTicks = ticks;
        
        PositionDutyCycle request = new PositionDutyCycle(leverTargetTicks).withSlot(0);
        lever.setControl(request);

        resetLeverTargetCounter();
    }
    public void setLeverState(State state) {
        double val;

        switch(state) {
            case ACTIVE:
            val = ClimbConstants.leverActiveDegrees;
            break;

            case RESTING:
            val = ClimbConstants.leverRestingDegrees;
            break;

            case UNKNOWN:
            default:
            DriverStation.reportWarning("Cannot rotate climb lever to UNKNOWN level", false);
            return;
        }

        setLeverDegrees(val);
    }
    public void leverVBus(double speed) {
        lever.set(speed);
        
        leverTargetTicks = Double.NaN;
        resetLeverTargetCounter();
    }
    public void leverStop() {
        leverVBus(0);
    }

    public double getLeverDegrees() {
        return getLeverTicks() * ClimbConstants.leverTickToDegConversion;
    }
    public double getLeverTicks() {
        return lever.getPosition().getValueAsDouble();
    }
    public State getLeverState() {
        return evalLeverState(getLeverDegrees());
    }
    public boolean isLeverIsAtTarget() {
        return leverIsAtTarget;
    }
    public boolean getLeverLimitSwitch() {
        return lever.getForwardLimit().getValue() == ForwardLimitValue.ClosedToGround;
    }

    public void setClampDegrees(double deg) {
        setClampTicks(deg / ClimbConstants.clampTickToDegConversion);
    }
    public void setClampTicks(double ticks) {
        if(ClimbConstants.clampUseSoftLimits)
            clampTargetTicks = MathUtil.clamp(ticks, ClimbConstants.clampMinPosTicks, ClimbConstants.clampMaxPosTicks);
        else
            clampTargetTicks = ticks;
        
        PositionDutyCycle request = new PositionDutyCycle(clampTargetTicks).withSlot(0);
        clamp.setControl(request);

        resetClampTargetCounter();
    }
    public void setClampState(State state) {
        double val;

        switch(state) {
            case ACTIVE:
            val = ClimbConstants.clampActiveDegrees;
            break;

            case RESTING:
            val = ClimbConstants.clampRestingDegrees;
            break;

            case UNKNOWN:
            default:
            DriverStation.reportWarning("Cannot rotate climb clamp to UNKNOWN level", false);
            return;
        }

        setClampDegrees(val);
    }
    public void clampVBus(double speed) {
        clamp.set(speed);
        
        clampTargetTicks = Double.NaN;
        resetClampTargetCounter();
    }
    public void clampStop() {
        leverVBus(0);
    }

    public double getClampDegrees() {
        return getClampTicks() * ClimbConstants.clampTickToDegConversion;
    }
    public double getClampTicks() {
        return clamp.getPosition().getValueAsDouble();
    }
    public State getClampState() {
        return evalClampState(getClampDegrees());
    }
    public boolean isClampIsAtTarget() {
        return clampIsAtTarget;
    }

    private void configureMotor(TalonFX m, double maxVbus, NeutralModeValue neutralMode, InvertedValue inverted, double maxCurrent, PIDFConfig pidf, double minTicks, double maxTicks, boolean useSoftLimits) {
        TalonFXConfiguration motorConfig = new TalonFXConfiguration();
        motorConfig.MotorOutput.PeakForwardDutyCycle = maxVbus;
        motorConfig.MotorOutput.PeakReverseDutyCycle = -maxVbus;
        motorConfig.MotorOutput.withNeutralMode(neutralMode);
        motorConfig.MotorOutput.Inverted = inverted;
        
        motorConfig.CurrentLimits.StatorCurrentLimit = maxCurrent;
        motorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        
        Slot0Configs slotConfigs = motorConfig.Slot0;
        slotConfigs.kS = slotConfigs.kV = 0;
        slotConfigs.kP = pidf.p;
        slotConfigs.kI = pidf.i;
        slotConfigs.kD = pidf.d;
        slotConfigs.kG = pidf.f;
        slotConfigs.GravityType = GravityTypeValue.Elevator_Static;
        
        m.getConfigurator().apply(motorConfig);
        
        SoftwareLimitSwitchConfigs softLimits = new SoftwareLimitSwitchConfigs();
        softLimits.ForwardSoftLimitEnable = softLimits.ReverseSoftLimitEnable = useSoftLimits;
        softLimits.ForwardSoftLimitThreshold = minTicks;
        softLimits.ReverseSoftLimitThreshold = maxTicks;
        m.getConfigurator().apply(softLimits);
    }
    private void resetLeverTargetCounter() {
        leverTargetCounter = 0;
        leverIsAtTarget = false; //to prevent a single frame where the target has been changed but the boolean hasnt been updated
    }
    private void resetClampTargetCounter() {
        clampTargetCounter = 0;
        clampIsAtTarget = false; //to prevent a single frame where the target has been changed but the boolean hasnt been updated
    }
    private State evalLeverState(double deg) {
        if(Math.abs(deg - ClimbConstants.leverActiveDegrees) <= ClimbConstants.targetThresholdDegrees)
            return State.ACTIVE;
        else if(Math.abs(deg - ClimbConstants.leverRestingDegrees) <= ClimbConstants.targetThresholdDegrees)
            return State.RESTING;
        
        return State.UNKNOWN;
    }
    private State evalClampState(double deg) {
        if(Math.abs(deg - ClimbConstants.clampActiveDegrees) <= ClimbConstants.targetThresholdDegrees)
            return State.ACTIVE;
        else if(Math.abs(deg - ClimbConstants.clampRestingDegrees) <= ClimbConstants.targetThresholdDegrees)
            return State.RESTING;
        
        return State.UNKNOWN;
    }
    private State getLeverTargetState() {
        return evalLeverState(leverTargetTicks * ClimbConstants.leverTickToDegConversion);
    }
    private State getClampTargetState() {
        return evalClampState(clampTargetTicks * ClimbConstants.clampTickToDegConversion);
    }
    /**
     * ONLY RUN ONCE PER UPDATE!!
     */
    private void leverTargetCheck() {
        double err = Math.abs(leverTargetTicks - getLeverTicks());
        if(err <= ClimbConstants.targetThresholdDegrees / ClimbConstants.leverTickToDegConversion)
            leverTargetCounter++;
        else
            resetLeverTargetCounter();
        
        if(leverTargetCounter >= ClimbConstants.targetThresholdSeconds * 50)
            leverIsAtTarget = true;
        else
            leverIsAtTarget = false;
    }
    private void clampTargetCheck() {
        double err = Math.abs(clampTargetTicks - getClampTicks());
        if(err <= ClimbConstants.targetThresholdDegrees / ClimbConstants.clampTickToDegConversion)
            clampTargetCounter++;
        else
            resetClampTargetCounter();
        
        if(clampTargetCounter >= ClimbConstants.targetThresholdSeconds * 50)
            clampIsAtTarget = true;
        else
            clampIsAtTarget = false;
    }
}
