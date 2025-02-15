package frc.robot.subsystems.elevator;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;




enum PresetPositions {
  barge,
  L4,
  L3,
  L2,
  L1,
  Processor,
  Indexer,
  HumanPlayer
}

enum rainbowPositions {
  topStop,
  aboveBarge,
  barge,
  belowBarge,
  betweenBargeAndL4,
  aboveL4,
  L4,
  belowL4,
  aboveL3,
  L3,
  belowL3,
  aboveL2,
  L2,
  belowL2,
  aboveL1,
  L1,
  belowL1,
  bottomStop
}

public class Elevator extends SubsystemBase {
  private SparkMax left_motor;
  private SparkMax right_motor;
  private PresetPositions targetPosition;
  private ColorSensorV3 rainbowEncoder;
  private final ColorMatch colorMatcher = new ColorMatch();
  private final Color topStopColor = new Color(.99, 0, 0.1);
  private final Color aboveBargeColor = new Color(0, 1, 0);
  // private final Color aboveBargeColor = new Color(0, 1, 0);






  private PIDController pidController;

  // PID Constants (Tune these values based on your robot's behavior)
  private static final double kP = 0.1;
  private static final double kI = 0.0;
  private static final double kD = 0.01;

  // Constructor
  public Elevator() {
    left_motor = new SparkMax(31, MotorType.kBrushless);
    right_motor = new SparkMax(32, MotorType.kBrushless);
    rainbowEncoder = new ColorSensorV3(Port.kMXP);

    pidController = new PIDController(kP, kI, kD);
    pidController.setTolerance(0.5); // Acceptable error margin


    // add colors to color matcher
    colorMatcher.addColorMatch(topStopColor);
    colorMatcher.addColorMatch(aboveBargeColor);

  }


  public void move(double speed) {
    left_motor.set(speed);
    right_motor.set(speed);
  }
  
  public void goToPosition(double targetPosition) {
    double currentPosition = getPosition();
    double output = pidController.calculate(currentPosition, targetPosition);
    left_motor.set(output);
    right_motor.set(-output);
  }

  public String matchColor(){ // converts sensor readings to color
    String colorString; 
    Color detectedColor = rainbowEncoder.getColor();
    ColorMatchResult match = colorMatcher.matchClosestColor(detectedColor);

    if (match.color == topStopColor) {
      colorString = "Blue";
    } else if (match.color == aboveBargeColor) {
      colorString = "Red";
    } else {
      colorString = "Unknown";
    } // TODO add logic for every color target 

    return colorString;
   
  } 


  public double getPosition(){ // converts color to position
    double currentPosition;
    if (matchColor() == "Blue") {
      currentPosition = 4;
    } else if (matchColor() == "Red") {
      currentPosition = 5;
    } else {
      currentPosition = 0;
    }

    return currentPosition;
  } // TODO map actual positions to colors AND do this for all color targets and positions listed in the enum

  

  public void periodic() {}
}
