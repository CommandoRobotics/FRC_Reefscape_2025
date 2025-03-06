package frc.robot.subsystems.elevator;


import java.util.function.DoubleSupplier;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


enum rainbowPositions { // MAKE SURE POSITIONS ARE LISTED FROM LOWEST TO HIGHEST
bottomStop, 
belowL1,
L1,
aboveL1,
belowL2,
L2,
aboveL2,
belowL3,
L3,
aboveL3,
belowL4,
L4,
aboveL4,
belowBarge,
barge,
aboveBarge,
topStop,
unknown
}


public class Elevator extends SubsystemBase {
 private SparkMax left_motor;
 private SparkMax right_motor;
 private ColorSensorV3 rainbowEncoder;
 private final ColorMatch colorMatcher = new ColorMatch();


 //*****ADD COLORS */   // TODO find out if these colors are actually practical
 private final Color topStopColor = new Color(194, 23, 163); // Magenta 16
 private final Color aboveBargeColor = new Color(193, 91, 245); // Light Purple 15
 private final Color bargeColor = new Color(130, 32, 176); // Purple 14
 private final Color belowBargeColor = new Color(38, 2, 99); // Dark Purple 13
 private final Color aboveL4Color = new Color(34, 6, 158); // Indigo 12
 private final Color L4Color = new Color(0,0,255); // Blue 11
 private final Color belowL4Color = new Color(0, 213, 255); // Sky Blue 10
 private final Color aboveL3Color = new Color(28, 212, 135); // Teal 9
 private final Color L3Color = new Color(0, 255, 0); // Green 8
 private final Color belowL3Color = new Color(203, 222, 31); // Yellow-Green 7
 private final Color aboveL2Color = new Color(242, 201, 36); // Yellow 6
 private final Color L2Color = new Color(214, 142, 34); // Orange-Yellow 5
 private final Color belowL2Color = new Color(235, 85, 40); // Orange 4
 private final Color aboveL1Color = new Color(160, 40, 5); // Dark Orange 3
 private final Color L1Color = new Color(255, 100, 100); // Pink 2
 private final Color belowL1Color = new Color(255, 0, 0); // Red 1
 private final Color bottomStopColor = new Color(125, 15, 15); // Dark Red 0

 private final Color red = new Color(1, 1, 1); //red
 private final Color orange = new Color(1, 1, 1); //orange
 private final Color darkYellow = new Color(1, 1, 1); //dark yellow
 private final Color lightYellow = new Color(1, 1, 1); //light yellow
 private final Color lightGreen = new Color(1, 1, 1); //light green
 private final Color green = new Color(1, 1, 1); //green
 private final Color darkGreen = new Color(1, 1, 1); //dark green
 private final Color blue = new Color(1, 1, 1); //blue
 private final Color purple = new Color(1, 1, 1); //purple
 private final Color magenta = new Color(1, 1, 1); //magenta purple
 private final Color lightBrown = new Color(1, 1, 1); //light brown
 private final Color darkBrown = new Color(1, 1, 1); //dark brown
 private final Color grey = new Color(1, 1, 1); //grey
 private final Color black = new Color(1, 1, 1); //black 
 private final Color white = new Color(1, 1, 1); //white 

 

 private final double elevatorUpSpeed = 0.8;
 private final double elevatorUpSlowSpeed = 0.4;

 private final double elevatorDownSpeed = -0.8;
 private final double elevatorDownSlowSpeed = -0.4;
 private final double elevatorLockSpeed = 0.05;


 // Constructor
 public Elevator() {
   left_motor = new SparkMax(31, MotorType.kBrushless);
   right_motor = new SparkMax(32, MotorType.kBrushless);
   rainbowEncoder = new ColorSensorV3(Port.kMXP);






   // add colors to color matcher
   colorMatcher.addColorMatch(topStopColor);
   colorMatcher.addColorMatch(aboveBargeColor);
   colorMatcher.addColorMatch(bargeColor);
   colorMatcher.addColorMatch(belowBargeColor);
   colorMatcher.addColorMatch(aboveL4Color);
   colorMatcher.addColorMatch(L4Color);
   colorMatcher.addColorMatch(belowL4Color);
   colorMatcher.addColorMatch(aboveL3Color);
   colorMatcher.addColorMatch(L3Color);
   colorMatcher.addColorMatch(belowL3Color);
   colorMatcher.addColorMatch(aboveL2Color);
   colorMatcher.addColorMatch(L2Color);
   colorMatcher.addColorMatch(belowL2Color);
   colorMatcher.addColorMatch(aboveL1Color);
   colorMatcher.addColorMatch(L1Color);
   colorMatcher.addColorMatch(belowL1Color);
   colorMatcher.addColorMatch(bottomStopColor);

   colorMatcher.addColorMatch(red);
   colorMatcher.addColorMatch(orange);
   colorMatcher.addColorMatch(darkYellow);
   colorMatcher.addColorMatch(lightYellow);
   colorMatcher.addColorMatch(lightGreen);
   colorMatcher.addColorMatch(green);
   colorMatcher.addColorMatch(darkGreen);
   colorMatcher.addColorMatch(blue);
   colorMatcher.addColorMatch(purple);
   colorMatcher.addColorMatch(magenta);
   colorMatcher.addColorMatch(lightBrown);
   colorMatcher.addColorMatch(darkBrown);
   colorMatcher.addColorMatch(grey);
   colorMatcher.addColorMatch(black);
   colorMatcher.addColorMatch(white);





 }


public void stop() {
  left_motor.stopMotor();
  right_motor.stopMotor();
}

public void lockElevator(){
  left_motor.set(elevatorLockSpeed);
  right_motor.set(-elevatorLockSpeed);

}


 public void move(double speed) {
   left_motor.set(speed);
   right_motor.set(-speed); // swap the negative if motors go the wrong direction
 }



 public void moveToDesiredPostion(int desiredPosition) { // moves elevator to desired position
  if (desiredPosition > getCurrentPosition()) {
    // moves elevator up
  if (desiredPosition - getCurrentPosition() > 1) {
    move(elevatorUpSpeed);
  } else {
    move(elevatorUpSlowSpeed);
  }

  } else if (desiredPosition < getCurrentPosition()) {
    // moves elevator down 
  if (desiredPosition - getCurrentPosition() < -1) {
    move(elevatorDownSpeed);
  } else {
    move(elevatorDownSlowSpeed);
  }

  } else { 
    // stops elevator
    move(elevatorLockSpeed);
  }
 }


 public int getCurrentPosition(){ // converts sensor readings to position indices
  int colorIndex;
   Color detectedColor = rainbowEncoder.getColor();
   ColorMatchResult match = colorMatcher.matchClosestColor(detectedColor);

   if (match.color == topStopColor) {
    colorIndex = rainbowPositions.topStop.ordinal();
   } else if (match.color == aboveBargeColor) {
    colorIndex = rainbowPositions.aboveBarge.ordinal();
   } else if (match.color == bargeColor) {
    colorIndex = rainbowPositions.barge.ordinal();
   } else if (match.color == belowBargeColor) {
    colorIndex = rainbowPositions.belowBarge.ordinal();
   } else if (match.color == aboveL4Color) {
    colorIndex = rainbowPositions.aboveL4.ordinal();
   } else if (match.color == L4Color) {
    colorIndex = rainbowPositions.L4.ordinal();
   } else if (match.color == belowL4Color) {
    colorIndex = rainbowPositions.belowL4.ordinal();
   } else if (match.color == aboveL3Color) {
    colorIndex = rainbowPositions.aboveL3.ordinal();
   } else if (match.color == L3Color) {
    colorIndex = rainbowPositions.L3.ordinal();
   } else if (match.color == belowL3Color) {
    colorIndex = rainbowPositions.belowL3.ordinal();
   } else if (match.color == aboveL2Color) {
    colorIndex = rainbowPositions.aboveL2.ordinal();
   } else if (match.color == L2Color) {
    colorIndex = rainbowPositions.L2.ordinal();
   } else if (match.color == belowL2Color) {
    colorIndex = rainbowPositions.belowL2.ordinal();
  } else if (match.color == aboveL1Color) {
    colorIndex = rainbowPositions.aboveL1.ordinal();
  } else if (match.color == L1Color) {
    colorIndex = rainbowPositions.L1.ordinal();
  } else if (match.color == belowL1Color) {
    colorIndex = rainbowPositions.belowL1.ordinal();
  } else if (match.color == bottomStopColor) {
    colorIndex = rainbowPositions.bottomStop.ordinal();
  } else {
    colorIndex = rainbowPositions.unknown.ordinal();
  }
   return colorIndex;
   }


   public String outputColorReading() {
    String seenColor;
    Color detectedColor = rainbowEncoder.getColor();
    ColorMatchResult match = colorMatcher.matchClosestColor(detectedColor);

    if (match.color == red) {
      seenColor = "red";
    } else if (match.color == orange) {
      seenColor = "orange";
    } else if (match.color == lightYellow) {
      seenColor = "lightyellow";
    } else if (match.color == darkYellow) {
      seenColor = "darkyellow";
    } else if (match.color == green) {
      seenColor = "green";
    } else if (match.color == lightGreen) {
      seenColor = "lightgreen";
    } else if (match.color == darkGreen) {
      seenColor = "dark green";
    } else if (match.color == blue) {
      seenColor = "blue";
    } else if (match.color == purple) {
      seenColor = "purple";
    } else if (match.color == magenta) {
      seenColor = "magenta";
    } else if (match.color == lightBrown) {
      seenColor = "light brown";
    } else if (match.color == darkBrown) {
      seenColor = "dark brown";
    } else if (match.color == grey) {
      seenColor = "grey";
    } else if (match.color == black) {
      seenColor = "black";
    } else if (match.color == white) {
      seenColor = "white";
    } else {
      seenColor = "unkown";
    }

    return seenColor;
  }
  
    

  //***************** COMMANDS **********************************************/

      // Use manual control. Power is -1.0 to +1.0
    public Command manualControlElevatorCommand(DoubleSupplier power) {
        return run( () -> move(power.getAsDouble()) );
    }


     public Command moveL1Command() {
        return run(() -> moveToDesiredPostion(rainbowPositions.L1.ordinal()));
    }
  
     public Command moveL2Command() {
      return run(() -> moveToDesiredPostion(rainbowPositions.L2.ordinal()));
    }

     public Command moveL3Command() {
      return run(() -> moveToDesiredPostion(rainbowPositions.L3.ordinal()));
    }

     public Command moveL4Command() {
      return run(() -> moveToDesiredPostion(rainbowPositions.L4.ordinal()));
    }

    public Command moveBargeCommand() {
      return run(() -> moveToDesiredPostion(rainbowPositions.barge.ordinal()));
    }

    public Command stopCommand() {
      return run(() -> stop());
    }
// *****************************************************************************

  @Override 
  public void periodic() {
    getCurrentPosition();  // updates current position of the robot every 20 ms


    outputColorReading();

    rainbowEncoder.getColor();

    Color seenColor = rainbowEncoder.getColor();
    double red = seenColor.red;
    double green = seenColor.green;
    double blue = seenColor.blue;

    SmartDashboard.putNumber("color red", red);
    SmartDashboard.putNumber("color green", green);
    SmartDashboard.putNumber("color blue", blue);

   // SmartDashboard.putString("seen color is: ", outputColorReading());


  }
 
 }




 

   







