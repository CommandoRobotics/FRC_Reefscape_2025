package frc.robot.subsystems.elevator;

import com.revrobotics.ColorSensorV3;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

enum presetPositions {
  barge,
  L4,
  L3,
  L2,
  L1,
  Indexer,
  Processor,
  HumanPlayer
};

enum rainbowPositions {
  topStop,
  aboveBarge,
  barge,
  belowBarge,
  betweenBargeAndL4,
  aboveL4,
  L4,
  belowL4,
  betweenL4andL3,
  // ContinueHere
  bottomStop
}

public class Elevator extends SubsystemBase {
  private SparkMax left_motor;
  private SparkMax right_motor;
  private presetPositions targetPosition;
  private ColorSensorV3 rainbowEncoder;
  private final Color topStopColor = new Color(0.99, 0, 0.1);
  private final Color aboveBargeColor = new Color(0, 1, 0);
  // Continue creating colors.

  // Constructor
  public Elevator() {
    left_motor = new SparkMax(31, MotorType.kBrushless);
    right_motor = new SparkMax(32, MotorType.kBrushless);
    rainbowEncoder = new ColorSensorV3(Port.kMXP);
  }

  public void move(double speed) {
    left_motor.set(speed);
    right_motor.set(speed);
  }

  public void gotoPosition(presetPositions position) {
    targetPosition = position;
  }

  public void periodic() {}
}
