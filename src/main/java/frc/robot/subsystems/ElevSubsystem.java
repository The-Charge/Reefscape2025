package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.ReverseLimitValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ElevConstants;
import frc.robot.constants.TelemetryConstants;

public class ElevSubsystem extends SubsystemBase {

    public enum Level {
        HOME,
        LVL1,
        LVL2,
        LVL3,
        LVL4,
        ALGAE_LOW,
        ALGAE_HIGH,
        UNKNOWN
    }
    
    private TalonFX motor;
    private double targetTicks = 0;
    private int targetCounter = 0;
    private boolean isAtTarget = true;
    private SendableChooser<Level> targetOverrideLvl;
    private boolean hardStopLast = true;

    public ElevSubsystem() {
        motor = new TalonFX(ElevConstants.motorID);

        configureMotor(motor);

        if(TelemetryConstants.debugTelemetry) {
            //used for smartdashboard override commands, read only
            SmartDashboard.putNumber(ElevConstants.overrideInName, 0);
            SmartDashboard.putNumber(ElevConstants.overrideTicksName, 0);

            targetOverrideLvl = new SendableChooser<>();
            targetOverrideLvl.addOption("Home", Level.HOME);
            targetOverrideLvl.addOption("Level 1", Level.LVL1);
            targetOverrideLvl.addOption("Level 2", Level.LVL2);
            targetOverrideLvl.addOption("Level 3", Level.LVL3);
            targetOverrideLvl.addOption("Level 4", Level.LVL4);
            targetOverrideLvl.addOption("Algae Low", Level.ALGAE_LOW);
            targetOverrideLvl.addOption("Algae High", Level.ALGAE_HIGH);
            targetOverrideLvl.setDefaultOption("Home", Level.HOME);
            SmartDashboard.putData(ElevConstants.overrideLVLName, targetOverrideLvl);
        }
    }

    @Override
    public void periodic() {
        if(ElevConstants.hardStopResetsEncoder)
            checkHardStop();
        
        targetCheck();

        if(getCurrentCommand() == null)
            SmartDashboard.putString("Elev RunningCommand", "None");
        else
            SmartDashboard.putString("Elev RunningCommand", getCurrentCommand().getName());
        
        SmartDashboard.putNumber("Elev Pos (In)", getPositionInches());
        SmartDashboard.putBoolean("Elev isAtTarget", isAtTarget());
        SmartDashboard.putBoolean("Elev HardStop", isAtHardStop());

        if(TelemetryConstants.debugTelemetry) {
            SmartDashboard.putString("Elev Pos (LVL)", getPositionLevel().name());
            SmartDashboard.putString("Elev Target (LVL)", getTargetLevel().name());

            SmartDashboard.putNumber("Elev Err (In)", (targetTicks - getPositionTicks()) * ElevConstants.tickToInConversion);
            SmartDashboard.putNumber("Elev Target (In)", targetTicks * ElevConstants.tickToInConversion);

            SmartDashboard.putNumber("Elev Pos (Ticks)", getPositionTicks());
            SmartDashboard.putNumber("Elev Err (Ticks)", targetTicks - getPositionTicks());
            SmartDashboard.putNumber("Elev Target (Ticks)", targetTicks);

            SmartDashboard.putNumber("Elev VBus", motor.get());
            SmartDashboard.putNumber("Elev Current", motor.getStatorCurrent().getValueAsDouble());
        }
    }

    public void setTargetPositionInches(double inches) {
        setTargetPositionTicks(inches / ElevConstants.tickToInConversion);
    }
    public void setTargetPositionTicks(double ticks) {
        targetTicks = MathUtil.clamp(ticks, ElevConstants.minPosTicks, ElevConstants.maxPosTicks);
        
        PositionDutyCycle request = new PositionDutyCycle(targetTicks).withSlot(0);
        motor.setControl(request);

        resetTargetCounter();
    }
    public void setTargetPositionLevel(Level lvl) {
        double val;

        switch(lvl) {
            case HOME:
            val = ElevConstants.homeInches;
            break;

            case LVL1:
            val = ElevConstants.lvl1Inches;
            break;

            case LVL2:
            val = ElevConstants.lvl2Inches;
            break;

            case LVL3:
            val = ElevConstants.lvl3Inches;
            break;

            case LVL4:
            val = ElevConstants.lvl4Inches;
            break;

            case ALGAE_LOW:
            val = ElevConstants.algaeLowInches;
            break;

            case ALGAE_HIGH:
            val = ElevConstants.algaeHighInches;
            break;

            case UNKNOWN:
            default:
            DriverStation.reportWarning("Cannot move elevator to UNKNOWN level", false);
            return;
        }

        setTargetPositionInches(val);
    }
    public void stop() {
        motor.set(0);
    }
    public void setAsZero() {
        motor.setPosition(0);
    }

    public double getPositionInches() {
        return getPositionTicks() * ElevConstants.tickToInConversion;
    }
    public double getPositionTicks() {
        return motor.getPosition().getValueAsDouble();
    }
    public Level getPositionLevel() {
        return getCurrentLevel(getPositionInches());
    }
    public boolean isAtTarget() {
        return isAtTarget;
    }
    public Level getOverrideLevel() {
        return targetOverrideLvl.getSelected();
    }
    public boolean isAtHardStop() {
        return motor.getReverseLimit().getValue() == ReverseLimitValue.ClosedToGround;
    }

    private void configureMotor(TalonFX m) {
        TalonFXConfiguration motorConfig = new TalonFXConfiguration();
        motorConfig.MotorOutput.PeakForwardDutyCycle = ElevConstants.maxVBus;
        motorConfig.MotorOutput.PeakReverseDutyCycle = -ElevConstants.maxVBus;
        motorConfig.MotorOutput.withNeutralMode(ElevConstants.neutralMode);
        motorConfig.MotorOutput.Inverted = ElevConstants.inverted;
        
        motorConfig.CurrentLimits.StatorCurrentLimit = ElevConstants.maxCurrent;
        motorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        
        Slot0Configs slotConfigs = motorConfig.Slot0;
        slotConfigs.kS = slotConfigs.kV = 0;
        slotConfigs.kP = ElevConstants.pidf.p;
        slotConfigs.kI = ElevConstants.pidf.i;
        slotConfigs.kD = ElevConstants.pidf.d;
        slotConfigs.kG = ElevConstants.pidf.f;
        slotConfigs.GravityType = GravityTypeValue.Elevator_Static;

        // conservative motionmagic configs
        var motionMagicConfigs = motorConfig.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = 20; // Target cruise velocity of 80 rps
        motionMagicConfigs.MotionMagicAcceleration = 80; // Target acceleration of 80 rps/s (1 second)
        motionMagicConfigs.MotionMagicJerk = 800; // Target jerk of 800 rps/s/s (0.2 seconds)

        // motorConfig.HardwareLimitSwitch.ReverseLimitAutosetPositionEnable = ElevConstants.hardStopResetsEncoder;
        // motorConfig.HardwareLimitSwitch.ReverseLimitAutosetPositionValue = ElevConstants.hardStopResetValue;
        
        m.getConfigurator().apply(motorConfig);
        
        SoftwareLimitSwitchConfigs softLimits = new SoftwareLimitSwitchConfigs();
        softLimits.ForwardSoftLimitEnable = softLimits.ReverseSoftLimitEnable = true;
        softLimits.ForwardSoftLimitThreshold = ElevConstants.maxPosTicks;
        softLimits.ReverseSoftLimitThreshold = ElevConstants.minPosTicks;
        m.getConfigurator().apply(softLimits);
    }
    private void resetTargetCounter() {
        targetCounter = 0;
        isAtTarget = false; //to prevent a single frame where the target has been changed but the boolean hasnt been updated
    }
    private Level getCurrentLevel(double inches) {
        if(inches <= ElevConstants.homeInches + ElevConstants.targetThresholdInches) //allow negatives to count as HOME
            return Level.HOME;
        else if(Math.abs(inches - ElevConstants.lvl1Inches) <= ElevConstants.targetThresholdInches)
            return Level.LVL1;
        else if(Math.abs(inches - ElevConstants.lvl2Inches) <= ElevConstants.targetThresholdInches)
            return Level.LVL2;
        else if(Math.abs(inches - ElevConstants.lvl3Inches) <= ElevConstants.targetThresholdInches)
            return Level.LVL3;
        else if(Math.abs(inches - ElevConstants.lvl4Inches) <= ElevConstants.targetThresholdInches)
            return Level.LVL4;
        else if(Math.abs(inches - ElevConstants.algaeLowInches) <= ElevConstants.targetThresholdInches)
            return Level.ALGAE_LOW;
        else if(Math.abs(inches - ElevConstants.algaeHighInches) <= ElevConstants.targetThresholdInches)
            return Level.ALGAE_HIGH;
        
        return Level.UNKNOWN;
    }
    private Level getTargetLevel() {
        return getCurrentLevel(targetTicks * ElevConstants.tickToInConversion);
    }
    /**
     * ONLY RUN ONCE PER UPDATE!!
     */
    private void targetCheck() {
        double err = Math.abs(targetTicks - getPositionTicks());
        if(err <= ElevConstants.targetThresholdInches / ElevConstants.tickToInConversion)
            targetCounter++;
        else
            resetTargetCounter();
        
        if(targetCounter >= ElevConstants.targetThresholdSeconds * 50)
            isAtTarget = true;
        else
            isAtTarget = false;
    }
    private void checkHardStop() {
        boolean hardStop = isAtHardStop();

        if(hardStop && !hardStopLast) {
            motor.setPosition(ElevConstants.hardStopResetValue);
        }

        hardStopLast = hardStop;
    }
}
