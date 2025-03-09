package frc.robot.subsystems.hook;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;

public class Hook extends SubsystemBase {

  private SparkMax hookMotor;
  private SparkMax rollerMotor;
  private RelativeEncoder hookEncoder;
  private RelativeEncoder rollerEncoder;

  private double rollerSpeed = 0.1;

  public Hook() {
    hookMotor = new SparkMax(61, SparkMax.MotorType.kBrushless);
    rollerMotor = new SparkMax(62, SparkMax.MotorType.kBrushless);

    hookEncoder = hookMotor.getEncoder();
    rollerEncoder = rollerMotor.getEncoder();
  }

  public void stop() { // stops the hand subsystem
    rollerMotor.set(0);
  }

  public void resetHookEncoder() {
    hookEncoder.setPosition(0);
  }

  public double hookPosition() {
    return hookEncoder.getPosition();
  }

  public double rollerRotations() {
    return rollerEncoder.getPosition();
  }

  public void manualHook(double power) {
    hookMotor.set(power);

    rollerMotor.set(0);
  }

  public void autoIntake() {

    rollerMotor.set(rollerSpeed);
  }

  public void Eject() {
    rollerMotor.set(-rollerSpeed);
  }

  public void removeAlgaeKick() {
    rollerMotor.set(rollerSpeed);
    hookMotor.set(1);
  }

  // ********************* COMMANDS ***************************/

  public Command stopCommand() { // stops all hand subsystem motors
    return run(() -> stop());
  }

  public Command resetHookCommand() { // stops all hand subsystem motors
    return run(() -> resetHookEncoder());
  }

  public Command manualControlHandCommand(DoubleSupplier power) {
    return run(() -> manualHook(power.getAsDouble()));
  }

  public Command hookIntakeCommand() {
    return run(() -> autoIntake());
  }

  public Command ejectAlgaeCommand() {
    return run(() -> Eject());
  }

  public Command kickAlgaeCommand() {
    return run(() -> removeAlgaeKick());
  }

  // ********************* END OF COMMANDS ***************************/

  @Override
  public void periodic() {
    // The arm is perpendicular to the Upright shoulder.

    SmartDashboard.putNumber("hook position rotations", hookPosition());
    SmartDashboard.putNumber("hook roller position rotations", rollerRotations());

    // SmartDashboard.putNumber("wrist position rotations", wristMotor.getEncoder().getPosition());
  }
}
