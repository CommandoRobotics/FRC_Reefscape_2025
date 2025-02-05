// Commando Robotics - FRC 5889
// Hand used for placing Coral
// Much of the code for this comes from the following example
// https://github.com/REVrobotics/REVLib-Examples/blob/main/Java/SPARK/Closed%20Loop%20Control/src/main/java/frc/robot/Robot.java

package frc.robot.subsystems.hand;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;

enum HandSetPoint {
  stowed, // Keeps hand within frame perimeter
  coralStation, // Intaking from human player station
  L1, // Trough
  L2, // Lowest branch of reef
  L3, // Middle branch on reef
  L4 // Top branch of reef
}

public class Hand extends SubsystemBase {
  // TODO: Tune Hand Hex encoder offset
  // Angle we want the hand to point (horizontal is zero degrees, up is positive).
  @AutoLogOutput private Rotation2d targetAngle;
  // Don't allow the hand to rotate up past this, or it will break (wires might yank).
  private final Rotation2d maxAngle = Rotation2d.fromDegrees(90);
  // Don't allow the hand to rotate down past this, or it will break (wires might yank).
  private final Rotation2d minAngle = Rotation2d.fromDegrees(-45);

  // Coral Motors
  final int leftCoralMotorCANID = 22;
  // Left (if scoring on reef is forward) motor to move Coral in/out of the hand.
  private SparkMax leftCoralMotor;

  // Wrist Motor
  final int wristMotorCANID = 21;
  private SparkMax wristMotor; // Motor for rotating the hand up and down.
  private SparkMaxConfig wristMotorConfig;
  private SparkClosedLoopController wristClosedLoopController;
  // Encoder is plugged directly into Wrist motor
  // Allows access to the attached Encoder
  private AbsoluteEncoder wristEncoder;
  // Angle (rotations) the HexboreEncoder reads when horizontal.
  private final double wristOffsetAngleInRotations = 0.0;
  // Variable to store the last read rotation of the motor.
  @AutoLogOutput private Rotation2d actualAngle;

  public Command stowCommand() {
    return this.runOnce(
        () -> {
          setTarget(HandSetPoint.stowed);
          stopCoral();
        });
  }

  public Command intakeFromHumanPlayerCommand() {
    return this.runOnce(
        () -> {
          setTarget(HandSetPoint.coralStation);
          reverseCoral();
        });
  }

  public Command bumpUpCommand() {
    return this.runOnce(() -> bumpUp());
  }

  // Constructor function
  public Hand() {
    targetAngle = Rotation2d.fromDegrees(0);
    leftCoralMotor = new SparkMax(leftCoralMotorCANID, MotorType.kBrushless);
    wristMotor = new SparkMax(wristMotorCANID, MotorType.kBrushless);
    wristClosedLoopController = wristMotor.getClosedLoopController();
    wristEncoder = wristMotor.getAbsoluteEncoder();
    // Configure the wrist motor
    wristMotorConfig = new SparkMaxConfig();
    wristMotorConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
        // Set PID values for position control. Deefault to slot 0.
        .p(0.1)
        .i(0)
        .d(0)
        .outputRange(minAngle.getRotations(), maxAngle.getRotations());

    // kResetSafeParameters is used to get the SPARK MAX to a known state. This is useful in case
    // the SPARK MAX is replaced.
    // kPersistParameters is used to ensure the configuration is not lost when  the SPARK MAX loses
    // power. This is useful for power cycles that may occur mid-operation.
    wristMotor.configure(
        wristMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    /// DEBUG CODE
    leftCoralMotor.set(1.0);
  }

  // Run Coral wheels so Coral should slide out front (score on reef)
  private void outputCoral() {
    leftCoralMotor.set(0.25);
  }

  // Run Coral wheels so they run backwards (intake Coral from Human player station)
  private void reverseCoral() {
    leftCoralMotor.set(-0.5);
  }

  private void stopCoral() {
    leftCoralMotor.stopMotor();
  }

  private void setTarget(HandSetPoint newTarget) {
    // ToDo: Adjust angles based on real angle the hand needs to be at
    double targetDegrees =
        switch (newTarget) {
          case stowed -> 0.0;
          case coralStation -> 55.0;
          case L1 -> -35.0;
          case L2 -> -35.0;
          case L4 -> -45.0;
          default -> 0.0;
        };
    targetAngle = Rotation2d.fromDegrees(targetDegrees);
  }

  // Returns true if the hand is at (or close enough) to the target angle, otherwise false.
  @AutoLogOutput
  private boolean checkIfAtTarget() {
    final double toleranceInDegrees = 5.0;
    Rotation2d delta = targetAngle.minus(actualAngle);
    return Math.abs(delta.getDegrees()) < toleranceInDegrees;
  }

  // Allows manually adjusting the targetAngle, rotating it down 1 degree.
  private void bumpDown() {
    manuallySetTarget(targetAngle.minus(Rotation2d.fromDegrees(1)));
  }

  // Allows manually adjusting the targetAngle, rotating it up 1 degree.
  private void bumpUp() {
    manuallySetTarget(targetAngle.plus(Rotation2d.fromDegrees(1)));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    actualAngle = Rotation2d.fromRotations(wristEncoder.getPosition());
    wristClosedLoopController.setReference(
        targetPositionAsMotorControllerSetPoint(), ControlType.kPosition, ClosedLoopSlot.kSlot0);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    Rotation2d delta = targetAngle.minus(actualAngle);
    // Speed to simulate the rotation (degrees per 100 times per second)
    final Rotation2d simulatedRotationSpeed = Rotation2d.fromDegrees(0.1);
    if (Math.abs(delta.getDegrees()) < simulatedRotationSpeed.getDegrees()) {
      // Close enough, place it at the target.
      actualAngle = targetAngle;
    } else if (delta.getDegrees() < 0) {
      // Move up
      actualAngle.plus(simulatedRotationSpeed);
    } else {
      actualAngle.minus(simulatedRotationSpeed);
    }
  }

  // Converts the targetAngle to a value the motor controller can use.
  // Returns the value as a percentage of rotation in the range 0 to 1.0
  private double targetPositionAsMotorControllerSetPoint() {
    Rotation2d adjustedTargetAngle =
        targetAngle.minus(Rotation2d.fromRotations(wristOffsetAngleInRotations));
    double targetAngleInRadians = adjustedTargetAngle.getRadians();
    double normalizedAngleInRadians = MathUtil.angleModulus(targetAngleInRadians);
    Rotation2d normalizedTargetAngle = Rotation2d.fromRadians(normalizedAngleInRadians);
    return normalizedTargetAngle.getDegrees() / 365.0;
  }

  // For setting a specific angle. Not for normal use.
  private void manuallySetTarget(Rotation2d newTarget) {
    if (newTarget.getDegrees() < minAngle.getDegrees()) {
      newTarget = minAngle;
    } else if (newTarget.getDegrees() > maxAngle.getDegrees()) {
      newTarget = maxAngle;
    }
    targetAngle = newTarget;
  }
}
