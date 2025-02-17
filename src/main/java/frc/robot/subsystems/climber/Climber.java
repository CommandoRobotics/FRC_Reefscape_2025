package frc.robot.subsystems.climber;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
    private SparkMax left_motor;
    private SparkMax right_motor;
    private DigitalInput climberLimitSwitch;
    private double climbSpeed = 0.6; // TODO tune this value so it works



    // Constructor
    public Climber() {
        left_motor = new SparkMax(71, MotorType.kBrushless);
        right_motor = new SparkMax(72, MotorType.kBrushless);
        climberLimitSwitch = new DigitalInput(55); // TODO Change this port number
    }


// **** METHODS ******

public void climbUp() {
  if (!climberLimitSwitch.get()) {   // runs climb motor until limit switch is pressed
    left_motor.set(-climbSpeed);
    right_motor.set(climbSpeed); 
} else {
    stopClimb();
}
}

public void stopClimb() {
    left_motor.set(0);
    right_motor.set(0);
}

//***** COMMANDS ********* */
public Command stopClimbCommand() {
    return run(() -> stopClimb());
}

public Command climbUpCommand() {
    return run(() -> climbUp());
}


}
