package frc.robot.subsystems.hand;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Hand {


public class Elevator extends SubsystemBase {
    private SparkMax left_motor;
    private SparkMax right_motor;
    private DigitalInput handBeamBreak;

    // private final Color aboveBargeColor = new Color(0, 1, 0);
    
    // Constructor
    public void Hand() {
        left_motor = new SparkMax(41, MotorType.kBrushless); // TODO Change this port number
        right_motor = new SparkMax(42, MotorType.kBrushless); // TODO Change this port number
        handBeamBreak = new DigitalInput(55); // TODO Change this port number


        }
    
        public void removeAlgae(double speed) {
            left_motor.set(speed);
            right_motor.set(speed);
        }

        public boolean beamBroken() {
            boolean beamBroken = handBeamBreak.get();
            return beamBroken;
        }
        
    }
      
    
}

