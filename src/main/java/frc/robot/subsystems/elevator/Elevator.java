package frc.robot.subsystems.elevator;

import com.revrobotics.ColorSensorV3;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
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
  // continue adding positions
  bottomStop
}

public class Elevator extends SubsystemBase {
  private SparkMax left_motor;
  private SparkMax right_motor;
  private PresetPositions targetPosition;
  private ColorSensorV3 rainbowEncoder;
  private final Color topStopColor = new Color(.99, 0, 0.1);
  private final Color aboveBargeColor = new Color(0, 1, 0);
  // private final Color aboveBargeColor = new Color(0, 1, 0);

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

  public void gotoPosition(PresetPositions position) {
    targetPosition = position;
  }

  public void periodic() {}
}
