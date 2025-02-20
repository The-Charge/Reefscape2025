package frc.robot.constants;

public abstract class AlgaeRemConstants {

    public static final int servoHubId = 0;
    public static final int servo1Id = 0;
    public static final int servo2Id = 1;
    public static final int flywheelId = 9;

    //using same min and max for both servos
    //https://www.andymark.com/products/programmable-servos#:~:text=am%2D4954%20High%20Torque%201%2DTurn
    public static final int servoMin = 500;
    public static final int servoMax = 2500;
    public static final boolean servo1Reversed = false;
    public static final boolean servo2Reversed = true;

    public static final int flywheelCurrentLimit = 25;

    public static final double servoOutPercent = 0.6;
    public static final double servoInPercent = 0;
    public static final double flywheelActiveVbus = 0.4;
    public static final double spinTime = 3; //seconds
}