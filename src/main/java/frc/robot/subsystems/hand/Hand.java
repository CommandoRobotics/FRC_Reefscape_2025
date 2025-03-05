package frc.robot.subsystems.hand;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Hand extends SubsystemBase {

  private SparkMax wristMotor;
  private SparkMax rollerMotor;
  private DigitalInput handBeamBreak;
  private SparkMaxConfig wristMotorConfig;
  private SparkAbsoluteEncoder wristEncoder;
  private SparkClosedLoopController wristClosedLoopController;

  private static final double kP = 0.05; // Tune these values as needed
  private static final double kI = 0.0;
  private static final double kD = 0.0;

  private static double wristEncoderOffsetRotations =
      0.0; // The encoder reads zero this many rotations from where we expect zero degrees to be.
  private static final double rollerSpeed = 0.5; // Speed of the roller motor
  private static final double minDegrees = 0; // Software limit switch // TODO tune this value
  private static final double maxDegrees = 100; // Software limit switch // TODO tune this value
  private static final double intakeHumanPlayerPositionDegrees =
      95; // position in degrees the hand needs to be to intake from human player // TODO tune this
  // value
  private static final double intakeFromIndexerPositionDegrees =
      10; // position in degrees the hand needs to be to intake from indexer // TODO tune this value
  private static final double L1PositionDegrees =
      0; // position in degrees the hand needs to be to score at L1 // TODO tune this value
  private static final double L2PositionDegrees =
      45; // position in degrees the hand needs to be to score at L2 // TODO tune this value
  private static final double L3PositionDegrees =
      45; // position in degrees the hand needs to be to score at L2 // TODO tune this value
  private static final double L4PositionDegrees =
      90; // position in degrees the hand needs to be to score at L2 // TODO tune this value

  public Hand() {
    wristMotor = new SparkMax(51, SparkMax.MotorType.kBrushless);
    rollerMotor = new SparkMax(53, SparkMax.MotorType.kBrushless);
    handBeamBreak = new DigitalInput(9); // TODO change port number
    // Configure the wrist motor
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
        .outputRange(
            Rotation2d.fromDegrees(minDegrees).getRotations(),
            Rotation2d.fromDegrees(maxDegrees).getRotations());

    // kResetSafeParameters is used to get the SPARK MAX to a known state. This is useful in case
    // the SPARK MAX is replaced.
    // kPersistParameters is used to ensure the configuration is not lost when  the SPARK MAX loses
    // power. This is useful for power cycles that may occur mid-operation.
    //  wristMotor.configure(
    //     wristMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  public void resetWristEncoder() { // resets the wrist encoder
    wristEncoderOffsetRotations = Rotation2d.fromRotations(wristEncoder.getPosition()).getDegrees();
  }

  private double targetPositionAsMotorControllerSetPoint(Rotation2d targetAngle) {
    Rotation2d adjustedTargetAngle =
        targetAngle.minus(Rotation2d.fromRotations(wristEncoderOffsetRotations));
    double targetAngleInRadians = adjustedTargetAngle.getRadians();
    double normalizedAngleInRadians = MathUtil.angleModulus(targetAngleInRadians);
    Rotation2d normalizedTargetAngle = Rotation2d.fromRadians(normalizedAngleInRadians);
    return normalizedTargetAngle.getDegrees() / 365.0;
  }

  public void setPosition(double newPositioninDegrees) {
    Rotation2d newRotation = Rotation2d.fromDegrees(newPositioninDegrees);
    wristClosedLoopController.setReference(
        targetPositionAsMotorControllerSetPoint(newRotation),
        ControlType.kPosition,
        ClosedLoopSlot.kSlot0);
  }

  public boolean coralInHand() { // checks if coral is fully captured
    return handBeamBreak.get();
  }

  public void stop() { // stops the hand subsystem
    wristMotor.set(0);
    rollerMotor.set(0);
  }

  public void autoIntakeHumanPlayer() { // automatically intakes coral from human player
    setPosition(intakeHumanPlayerPositionDegrees);
    if (!coralInHand()) {
      rollerMotor.set(rollerSpeed);
    } else {
      rollerMotor.set(0);
    }
  }

  public void autoIntakeIndexer() { // automatically intakes coral from indexer
    setPosition(intakeFromIndexerPositionDegrees);
    if (!coralInHand()) {
      rollerMotor.set(-rollerSpeed);
    } else {
      rollerMotor.set(0);
    }
  }

  public void expelCoral() { // expels coral
    rollerMotor.set(-rollerSpeed);
  }

  public void goToScorePostion(String Level) { // scores coral TAKES INPUT of L1, L2, L3, or L4
    if (Level == "L1") {
      setPosition(L1PositionDegrees);
    } else if (Level == "L2") {
      setPosition(L2PositionDegrees);
    } else if (Level == "L3") {
      setPosition(L3PositionDegrees);
    } else if (Level == "L4") {
      setPosition(L4PositionDegrees);
    } else {
      setPosition(0);
    }
  }

  // ********************* COMMANDS ***************************/

  public Command stopCommand() { // stops all hand subsystem motors
    return run(() -> stop());
  }

  public Command setPositionCommand(
      double targetPosition) { // runs the wrist motor until the target position is reached
    return run(() -> setPosition(targetPosition));
  }

  public Command autoIntakeHumanPlayerCommand() { // automatically intakes coral from human player
    return run(() -> autoIntakeHumanPlayer());
  }

  public Command autoIntakeIndexerCommand() { // automatically intakes coral from indexer
    return run(() -> autoIntakeIndexer());
  }

  public Command expelCoralCommand() { // expels coral
    return run(() -> expelCoral());
  }

  public Command goToScorePostionCommand(
      String Level) { // scores coral TAKES INPUT of L1, L2, L3, or L4
    return run(() -> goToScorePostion(Level));
  }

  // ********************* END OF COMMANDS ***************************/

  @Override
  public void periodic() {
    // The arm is perpendicular to the Upright shoulder.
    SmartDashboard.putBoolean("hand detects coral", coralInHand());

    double actualDegrees = Rotation2d.fromRotations(wristEncoder.getPosition()).getDegrees();
    SmartDashboard.putNumber("Wrist Angle", actualDegrees);

    double motorPosition =
        34
            + (Rotation2d.fromRotations(wristMotor.getAlternateEncoder().getPosition()).getDegrees()
                / 2);

    SmartDashboard.putNumber("Wrist Angle Debug (degrees)", motorPosition);
  }

  @Override
  public void simulationPeriodic() {
    // The arm is perpendicular to the Upright shoulder.
    setPosition(L3PositionDegrees);
    // SmartDashboard.putBoolean("hand detects coral", coralInHand());
    // double currentAngle = Rotation2d.fromRotations(wristEncoder.getPosition()).getDegrees();
    // SmartDashboard.putNumber("Wrist Angle (degrees)", currentAngle);
  }
}
