package frc.robot.subsystems.elevator;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;

enum rainbowPositions { // MAKE SURE POSITIONS ARE LISTED FROM LOWEST TO HIGHEST
  L2,
  aboveL2,
  belowL3,
  L3,
  aboveL3,
  belowL4,
  L4,
  aboveL4,
  belowBarge,
  unknown
}

public class Elevator extends SubsystemBase {
  private SparkMax left_motor;
  private SparkMax right_motor;
  private ColorSensorV3 rainbowEncoder;
  private final ColorMatch colorMatcher = new ColorMatch();

  // *****ADD COLORS */   // TODO find out if these colors are actually practical

  private final Color red = new Color(0.61694, 0.298828, 0.08569); // red
  private final Color orange = new Color(0.517, 0.403, 0.07959); // orange
  private final Color lightOrange = new Color(0.456, 0.4819, 0.062256); // light orange // NEW COLOR

  private final Color darkYellow = new Color(0.3562, 0.562012, 0.082031); // dark yellow
  private final Color lightYellow = new Color(0.30468, 0.548828, 0.146973); // light yellow
  private final Color lightGreen = new Color(0.203, 0.612, 0.184); // light green
  private final Color green = new Color(0.102, .624, 0.273); // green
  private final Color darkGreen = new Color(0.179688, 0.5273, 0.29269); // dark green
  private final Color blue = new Color(0.1145, 0.3237, 0.561); // blue

  private final Color lightBlue = new Color(0.1743, 0.44116, 0.38476); // light blue NEW COLOR

  private final Color purple = new Color(0.2326, 0.3093, 0.4587); // purple
  private final Color magenta = new Color(0.4162, 0.3095, 0.2754); // magenta purple

  private final Color pink = new Color(0.428, 0.3496, 0.2226); // pink NEW COLOR

  private final Color lightBrown = new Color(0.375, 0.4448, 0.181); // light brown
  private final Color darkBrown = new Color(0.307, 0.473, 0.221); // dark brown
  private final Color grey = new Color(0.251, 0.478, 0.2712); // grey
  private final Color black = new Color(0.232, 0.477, 0.290); // black
  private final Color white = new Color(0.253, 0.48, 0.265381); // white

  private final Color belowBargeColor = purple; // 8
  private final Color aboveL4Color = lightBlue; // 7
  private final Color L4Color = blue; // 6
  private final Color belowL4Color = lightGreen; // 5
  private final Color aboveL3Color = green; // 4
  private final Color L3Color = darkYellow; // 3
  private final Color belowL3Color = lightOrange; // 2
  private final Color aboveL2Color = orange; // 1
  private final Color L2Color = red; // 0 (bottom of robot)

  private final double elevatorUpSpeed = -0.3;
  private final double elevatorUpSlowSpeed = -0.15;

  private final double elevatorDownSpeed = 0.15;
  private final double elevatorDownSlowSpeed = 0.075;
  private final double elevatorLockSpeed = 0.05;

  // Constructor
  public Elevator() {
    left_motor = new SparkMax(31, MotorType.kBrushless);
    right_motor = new SparkMax(32, MotorType.kBrushless);
    rainbowEncoder = new ColorSensorV3(Port.kMXP);

    // add colors to color matcher

    colorMatcher.addColorMatch(belowBargeColor);
    colorMatcher.addColorMatch(aboveL4Color);
    colorMatcher.addColorMatch(L4Color);
    colorMatcher.addColorMatch(belowL4Color);
    colorMatcher.addColorMatch(aboveL3Color);
    colorMatcher.addColorMatch(L3Color);
    colorMatcher.addColorMatch(belowL3Color);
    colorMatcher.addColorMatch(aboveL2Color);
    colorMatcher.addColorMatch(L2Color);

    colorMatcher.addColorMatch(red);
    colorMatcher.addColorMatch(orange);
    colorMatcher.addColorMatch(darkYellow);
    colorMatcher.addColorMatch(lightGreen);
    colorMatcher.addColorMatch(green);
    colorMatcher.addColorMatch(blue);
    colorMatcher.addColorMatch(purple);
    colorMatcher.addColorMatch(lightOrange);
    colorMatcher.addColorMatch(lightBlue);
  }

  public void stop() {
    left_motor.stopMotor();
    right_motor.stopMotor();
  }

  public void lockElevator() {
    left_motor.set(elevatorLockSpeed);
    right_motor.set(-elevatorLockSpeed);
  }

  public void move(double speed) {
    left_motor.set(speed - .07);
    right_motor.set(-speed + .07); // swap the negative if motors go the wrong direction
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

  public int getCurrentPosition() { // converts sensor readings to position indices
    int colorIndex;
    Color detectedColor = rainbowEncoder.getColor();
    ColorMatchResult match = colorMatcher.matchClosestColor(detectedColor);

    if (match.color == belowBargeColor) {
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
    } else if (match.color == lightOrange) {
      seenColor = "light orange";
    } else if (match.color == lightBlue) {
      seenColor = "light blue";
    } else if (match.color == pink) {
      seenColor = "pink";
    } else {
      seenColor = "unkown";
    }

    return seenColor;
  }

  public String outputHeightReading() {
    String seenColor;
    Color detectedColor = rainbowEncoder.getColor();
    ColorMatchResult match = colorMatcher.matchClosestColor(detectedColor);

    if (match.color == belowBargeColor) {
      seenColor = "below barge";
    } else if (match.color == aboveL4Color) {
      seenColor = "above L4";
    } else if (match.color == L4Color) {
      seenColor = "at L4";
    } else if (match.color == belowL4Color) {
      seenColor = "below L4";
    } else if (match.color == aboveL3Color) {
      seenColor = "above L3";
    } else if (match.color == L3Color) {
      seenColor = "at L3";
    } else if (match.color == belowL3Color) {
      seenColor = "below L3";
    } else if (match.color == aboveL2Color) {
      seenColor = "above L2 ";
    } else if (match.color == L2Color) {
      seenColor = "at L2";
    } else {
      seenColor = "unkown";
    }

    return seenColor; // TODO add rest of heights
  }

  // ***************** COMMANDS **********************************************/

  // Use manual control. Power is -1.0 to +1.0
  public Command manualControlElevatorCommand(DoubleSupplier power) {
    return run(() -> move(power.getAsDouble()));
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

  public Command stopCommand() {
    return run(() -> stop());
  }
  // *****************************************************************************

  @Override
  public void periodic() {
    getCurrentPosition(); // updates current position of the robot every 20 ms

    outputColorReading();

    rainbowEncoder.getColor();

    Color seenColor = rainbowEncoder.getColor();
    double red = seenColor.red;
    double green = seenColor.green;
    double blue = seenColor.blue;

    SmartDashboard.putNumber("color red", red);
    SmartDashboard.putNumber("color green", green);
    SmartDashboard.putNumber("color blue", blue);

    SmartDashboard.putString("seen color is: ", outputColorReading());
  }
}
