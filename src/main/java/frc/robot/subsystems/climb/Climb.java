package frc.robot.subsystems.climb;

import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;

public class Climb extends SubsystemBase {

  private SparkMax leftClimb;
  private SparkMax rightClimb;

  public Climb() {
    leftClimb = new SparkMax(21, SparkMax.MotorType.kBrushless);
    rightClimb = new SparkMax(22, SparkMax.MotorType.kBrushless);
  }

  public void stop() { // stops the hand subsystem
    leftClimb.set(0);
    rightClimb.set(0);
  }

  public void manualClimb(double power) {
    rightClimb.set(power);
    leftClimb.set(-power);
  }

  // ********************* COMMANDS ***************************/

  public Command stopCommand() { // stops all hand subsystem motors
    return run(() -> stop());
  }

  public Command manualControlClimbCommand(DoubleSupplier power) {
    return run(() -> manualClimb(power.getAsDouble()));
  }

  // ********************* END OF COMMANDS ***************************/

  @Override
  public void periodic() {
    // The arm is perpendicular to the Upright shoulder.

    // SmartDashboard.putNumber("wrist position rotations", wristMotor.getEncoder().getPosition());
  }
}
