package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {
    
    AddressableLEDBuffer Buffer; // Define here
    AddressableLED Led;
    LEDPattern blackLedPattern = LEDPattern.solid(Color.kBlack); // this is off


    LEDPattern Green = LEDPattern.solid(Color.kRed); // this is actually green not red
    public void Stop() {
        blackLedPattern.applyTo(Buffer);
        Led.setData(Buffer);

    }

    public void  LEDs() {
        Green.applyTo(Buffer);
        Led.setData(Buffer);
    }

    public LEDSubsystem() {
        Led = new AddressableLED(0); // only put variable here when initializing
        Buffer = new AddressableLEDBuffer(44); // initialize here
        Led.setLength(Buffer.getLength());
//     m_led.start(); // to start
        Led.start();
    }

    @Override
    public void periodic() {
        
    }
}
