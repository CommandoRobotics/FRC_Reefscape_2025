package frc.robot.subsystems.handmanual;

import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;

public class HandManual extends SubsystemBase {

  private SparkMax wristMotor;
  private SparkMax rollerMotor;
  private DigitalInput handBeamBreak;
  private SparkMaxConfig wristMotorConfig;
  private SparkAbsoluteEncoder wristEncoder;
  private DutyCycleEncoder wristEncoder2;
  private SparkClosedLoopController wristClosedLoopController;

  private static final double kP = 12; // Tune these values as needed
  private static final double kI = 0.0;
  private static final double kD = 0.0;

  private static double wristEncoderOffsetRotations =
      0; // The encoder reads zero this many rotations from where we expect zero degrees to be.
  private static final double rollerSpeed = 0.5; // Speed of the roller motor
  private static final double minDegrees = -181; // Software limit switch // TODO tune this value
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

  public HandManual() {
    wristMotor = new SparkMax(61, SparkMax.MotorType.kBrushless);
    rollerMotor = new SparkMax(53, SparkMax.MotorType.kBrushless);
    handBeamBreak = new DigitalInput(9); // TODO change port number
    // Configure the wrist motor
    wristClosedLoopController = wristMotor.getClosedLoopController();
    wristEncoder = wristMotor.getAbsoluteEncoder();

    // Configure the wrist motor
    wristMotorConfig = new SparkMaxConfig();

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

  public boolean coralInHand() { // checks if coral is fully captured
    return handBeamBreak.get();
  }

  public void stop() { // stops the hand subsystem
    wristMotor.set(0);
    rollerMotor.set(0);
  }

  public void manualWrist(DoubleSupplier wristPower) {
    wristMotor.set(wristPower.getAsDouble());
  }

  // ********************* COMMANDS ***************************/

  public Command stopCommand() { // stops all hand subsystem motors
    return run(() -> stop());
  }

  public Command manualWristCommand(DoubleSupplier wristPower) { // stops all hand subsystem motors
    return run(() -> manualWrist(wristPower));
  }

  // ********************* END OF COMMANDS ***************************/

  @Override
  public void periodic() {
    // The arm is perpendicular to the Upright shoulder.
    SmartDashboard.putBoolean("hand detects coral", coralInHand());

    double actualDegrees = Rotation2d.fromRotations(wristEncoder.getPosition()).getDegrees();
    SmartDashboard.putNumber("Wrist Angle", actualDegrees);

    double motorPosition =
        (Rotation2d.fromRotations(wristMotor.getAlternateEncoder().getPosition()).getDegrees() / 2);

    SmartDashboard.putNumber("Wrist Angle Debug (degrees)", motorPosition);
  }

  @Override
  public void simulationPeriodic() {
    // The arm is perpendicular to the Upright shoulder.
    // SmartDashboard.putBoolean("hand detects coral", coralInHand());
    // double currentAngle = Rotation2d.fromRotations(wristEncoder.getPosition()).getDegrees();
    // SmartDashboard.putNumber("Wrist Angle (degrees)", currentAngle);
  }
}
