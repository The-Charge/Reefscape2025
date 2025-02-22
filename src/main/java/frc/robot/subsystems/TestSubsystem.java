package frc.robot.subsystems;


import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TestSubsystem extends SubsystemBase {
    TalonFX Motor;
    public void stop(){
        Motor.set(0);
    }
    public void set(){
        Motor.set(0.4);
    }

    public TestSubsystem() {
        Motor = new TalonFX(5);
        

   
   
    }
}
