package frc.robot.commands;

import java.util.HashMap;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.datalog.IntegerLogEntry;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.util.datalog.StructArrayLogEntry;
import edu.wpi.first.util.datalog.StructLogEntry;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.constants.TelemetryConstants;

public abstract class LoggingManager {
    
    private static final HashMap<String, BooleanLogEntry> boolEntries;
    private static final HashMap<String, DoubleLogEntry> doubleEntries;
    @SuppressWarnings("rawtypes")
    private static final HashMap<String, StructLogEntry> structEntries; //use raw types because generics suck
    @SuppressWarnings("rawtypes")
    private static final HashMap<String, StructArrayLogEntry> structArrayEntries;
    private static final HashMap<String, StringLogEntry> stringEntries;
    private static final HashMap<String, IntegerLogEntry> intEntries;

    static {
        boolEntries = new HashMap<>();
        doubleEntries = new HashMap<>();
        structEntries = new HashMap<>();
        structArrayEntries = new HashMap<>();
        stringEntries = new HashMap<>();
        intEntries = new HashMap<>();

        logValue("BumperPose", Pose3d.struct, Pose3d.kZero); //one time, needed for 3D field view
    }

    /**
     * Logs a value to the on-robot log file. Note that the key automatically has "/AvScope/" appended to the beginning
     */
    public static void logValue(String key, boolean val) {
        logValue(key, val, true);
    }
    /**
     * Logs a value to the on-robot log file. Note that the key automatically has "/AvScope/" appended to the beginning
     */
    public static void logValue(String key, double val) {
        logValue(key, val, true);
    }
    /**
     * Logs a value to the on-robot log file. Note that the key automatically has "/AvScope/" appended to the beginning
     */
    public static <T> void logValue(String key, Struct<T> struct, T val) {
        logValue(key, struct, val, true);
    }
    /**
     * Logs a value to the on-robot log file. Note that the key automatically has "/AvScope/" appended to the beginning
     */
    public static <T> void logValue(String key, Struct<T> struct, T[] vals) {
        logValue(key, struct, vals, true);
    }
    /**
     * Logs a value to the on-robot log file. Note that the key automatically has "/AvScope/" appended to the beginning
     */
    public static void logValue(String key, String val) {
        logValue(key, val, true);
    }
    /**
     * Logs a value to the on-robot log file. Note that the key automatically has "/AvScope/" appended to the beginning
     */
    public static void logValue(String key, int val) {
        logValue(key, val, true);
    }

    /**
     * Logs a value to the on-robot log file. Note that the key automatically has "/AvScope/" appended to the beginning
     */
    public static void logValue(String key, boolean val, boolean ignoreRepeats) {
        if(DataLogManager.getLogDir().isEmpty()) {
            DriverStation.reportWarning("Attempted to log to file before logging has started", false);
            return;
        }

        String newKey = "/AvScope/" + key; //unsafe, I don't care

        if(!boolEntries.containsKey(newKey)) {
            BooleanLogEntry entry = new BooleanLogEntry(DataLogManager.getLog(), newKey);
            entry.update(val); //no chance of repeat value but still need to use update rather than append in order to update last value

            boolEntries.put(newKey, entry);
            return;
        }

        BooleanLogEntry entry = boolEntries.get(newKey);
        if(ignoreRepeats) {
            entry.update(val); //update will check if the values has changed before logging
            return;
        }
        entry.append(val);
    }
    /**
     * Logs a value to the on-robot log file. Note that the key automatically has "/AvScope/" appended to the beginning
     */
    public static void logValue(String key, double val, boolean ignoreRepeats) {
        if(DataLogManager.getLogDir().isEmpty()) {
            DriverStation.reportWarning("Attempted to log to file before logging has started", false);
            return;
        }

        String newKey = "/AvScope/" + key; //unsafe, I don't care

        if(!doubleEntries.containsKey(newKey)) {
            DoubleLogEntry entry = new DoubleLogEntry(DataLogManager.getLog(), newKey);
            entry.update(val); //no chance of repeat value but still need to use update rather than append in order to update last value

            doubleEntries.put(newKey, entry);
            return;
        }

        DoubleLogEntry entry = doubleEntries.get(newKey);
        if(ignoreRepeats) {
            entry.update(val); //update will check if the values has changed before logging
            return;
        }
        entry.append(val);
    }
    /**
     * Logs a value to the on-robot log file. Note that the key automatically has "/AvScope/" appended to the beginning
     */
    @SuppressWarnings("unchecked")
    public static <T> void logValue(String key, Struct<T> struct, T val, boolean ignoreRepeats) {
        if(DataLogManager.getLogDir().isEmpty()) {
            DriverStation.reportWarning("Attempted to log to file before logging has started", false);
            return;
        }

        String newKey = "/AvScope/" + key; //unsafe, I don't care

        if(!structEntries.containsKey(newKey)) {
            StructLogEntry<T> entry = StructLogEntry.create(DataLogManager.getLog(), newKey, struct);
            entry.update(val); //no chance of repeat value but still need to use update rather than append in order to update last value

            structEntries.put(newKey, entry);
            return;
        }

        //don't know if a try catch will even work, doing it anyways
        try {
            StructLogEntry entry = structEntries.get(newKey);
            if(ignoreRepeats) {
                entry.update(val); //update will check if the values has changed before logging
                return;
            }
            entry.append(val);
        }
        catch(Exception e) {
            DriverStation.reportError("Raw type exception in logValue of Struct", e.getStackTrace());
        }
    }
    /**
     * Logs a value to the on-robot log file. Note that the key automatically has "/AvScope/" appended to the beginning
     */
    @SuppressWarnings("unchecked")
    public static <T> void logValue(String key, Struct<T> struct, T[] vals, boolean ignoreRepeats) {
        if(DataLogManager.getLogDir().isEmpty()) {
            DriverStation.reportWarning("Attempted to log to file before logging has started", false);
            return;
        }

        String newKey = "/AvScope/" + key; //unsafe, I don't care

        if(!structArrayEntries.containsKey(newKey)) {
            StructArrayLogEntry<T> entry = StructArrayLogEntry.create(DataLogManager.getLog(), newKey, struct);
            entry.update(vals); //no chance of repeat value but still need to use update rather than append in order to update last value

            structArrayEntries.put(newKey, entry);
            return;
        }

        //don't know if a try catch will even work, doing it anyways
        try {
            StructArrayLogEntry entry = structArrayEntries.get(newKey);
            if(ignoreRepeats) {
                entry.update(vals); //update will check if the values has changed before logging
                return;
            }
            entry.append(vals);
        }
        catch(Exception e) {
            DriverStation.reportError("Raw type exception in logValue of StructArray", e.getStackTrace());
        }
    }
    /**
     * Logs a value to the on-robot log file. Note that the key automatically has "/AvScope/" appended to the beginning
     */
    public static void logValue(String key, String val, boolean ignoreRepeats) {
        if(DataLogManager.getLogDir().isEmpty()) {
            DriverStation.reportWarning("Attempted to log to file before logging has started", false);
            return;
        }

        String newKey = "/AvScope/" + key; //unsafe, I don't care

        if(!stringEntries.containsKey(newKey)) {
            StringLogEntry entry = new StringLogEntry(DataLogManager.getLog(), newKey);
            entry.update(val); //no chance of repeat value but still need to use update rather than append in order to update last value

            stringEntries.put(newKey, entry);
            return;
        }

        StringLogEntry entry = stringEntries.get(newKey);
        if(ignoreRepeats) {
            entry.update(val); //update will check if the values has changed before logging
            return;
        }
        entry.append(val);
    }
    /**
     * Logs a value to the on-robot log file. Note that the key automatically has "/AvScope/" appended to the beginning
     */
    public static void logValue(String key, int val, boolean ignoreRepeats) {
        if(DataLogManager.getLogDir().isEmpty()) {
            DriverStation.reportWarning("Attempted to log to file before logging has started", false);
            return;
        }

        String newKey = "/AvScope/" + key; //unsafe, I don't care

        if(!intEntries.containsKey(newKey)) {
            IntegerLogEntry entry = new IntegerLogEntry(DataLogManager.getLog(), newKey);
            entry.update(val); //no chance of repeat value but still need to use update rather than append in order to update last value

            intEntries.put(newKey, entry);
            return;
        }

        IntegerLogEntry entry = intEntries.get(newKey);
        if(ignoreRepeats) {
            entry.update(val); //update will check if the values has changed before logging
            return;
        }
        entry.append(val);
    }

    /**
     * Logs a value to the on-robot log file and sends it to the network tables
     */
    public static void logAndSendValue(String key, boolean val) {
        SmartDashboard.putBoolean(key, val);
        logValue(key, val);
    }
    /**
     * Logs a value to the on-robot log file and sends it to the network tables
     */
    public static void logAndSendValue(String key, double val) {
        SmartDashboard.putNumber(key, val);
        logValue(key, val);
    }
    /**
     * Logs a value to the on-robot log file and sends it to the network tables
     */
    public static void logAndSendValue(String key, String val) {
        SmartDashboard.putString(key, val);
        logValue(key, val);
    }
    /**
     * Logs a value to the on-robot log file and sends it to the network tables
     */
    public static void logAndSendValue(String key, int val) {
        SmartDashboard.putNumber(key, val);
        logValue(key, val);
    }

    /**
     * Logs a value to the on-robot log file and sends it to the network tables if {@link frc.robot.constants.TelemetryConstants#debugTelemetry TelemetryConstants.debugTelemetry} is true
     */
    public static void logAndAutoSendValue(String key, boolean val) {
        if(TelemetryConstants.debugTelemetry) {
            logAndSendValue(key, val);
            return;
        }

        logValue(key, val);
    }
    /**
     * Logs a value to the on-robot log file and sends it to the network tables if {@link frc.robot.constants.TelemetryConstants#debugTelemetry TelemetryConstants.debugTelemetry} is true
     */
    public static void logAndAutoSendValue(String key, double val) {
        if(TelemetryConstants.debugTelemetry) {
            logAndSendValue(key, val);
            return;
        }

        logValue(key, val);
    }
    /**
     * Logs a value to the on-robot log file and sends it to the network tables if {@link frc.robot.constants.TelemetryConstants#debugTelemetry TelemetryConstants.debugTelemetry} is true
     */
    public static void logAndAutoSendValue(String key, String val) {
        if(TelemetryConstants.debugTelemetry) {
            logAndSendValue(key, val);
            return;
        }

        logValue(key, val);
    }
    /**
     * Logs a value to the on-robot log file and sends it to the network tables if {@link frc.robot.constants.TelemetryConstants#debugTelemetry TelemetryConstants.debugTelemetry} is true
     */
    public static void logAndAutoSendValue(String key, int val) {
        if(TelemetryConstants.debugTelemetry) {
            logAndSendValue(key, val);
            return;
        }

        logValue(key, val);
    }
}
