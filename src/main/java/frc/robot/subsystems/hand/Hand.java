package frc.robot.subsystems.hand;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;

public class Hand extends SubsystemBase {

  private SparkMax wristMotor;
  private SparkMax rollerMotor;
  private DigitalInput frontBeamBreak;
  private DigitalInput backBeamBreak;
  private DigitalInput handBeamBreak2;

  private SparkMaxConfig wristMotorConfig;
  private AbsoluteEncoder wristEncoder;
  private SparkClosedLoopController wristClosedLoopController;

  private double rollerSpeed = 0.5;

  public Hand() {
    wristMotor = new SparkMax(51, SparkMax.MotorType.kBrushless);
    rollerMotor = new SparkMax(53, SparkMax.MotorType.kBrushless);
    frontBeamBreak = new DigitalInput(0); // TODO change port number
    backBeamBreak = new DigitalInput(1); // TODO change port number
  }

  public boolean frontBeamBreakDetectsCoral() { // checks if coral is fully captured
    return !frontBeamBreak.get();
  }

  public boolean backBeamBreakDetectsCoral() { // checks if coral is fully captured
    return !backBeamBreak.get();
  }

  public void stop() { // stops the hand subsystem
    wristMotor.set(0);
    rollerMotor.set(0);
  }

  public void manualHand(DoubleSupplier power) {
    wristMotor.set(power.getAsDouble());
  }

  public void autoIntake() {
    if (frontBeamBreakDetectsCoral() && !backBeamBreakDetectsCoral()) {
      rollerMotor.set(rollerSpeed * .5);
    } else if (backBeamBreakDetectsCoral()) {
      rollerMotor.set(0);
    } else {
      rollerMotor.set(rollerSpeed);
    }
  }

  public void primeEject() {
    if (frontBeamBreakDetectsCoral() && backBeamBreakDetectsCoral()) {
      rollerMotor.set(-.1);
    } else {
      rollerMotor.set(0);
    }
  }

  public void Eject() {
    rollerMotor.set(-rollerSpeed);
  }

  // ********************* COMMANDS ***************************/

  public Command stopCommand() { // stops all hand subsystem motors
    return run(() -> stop());
  }

  public Command manualControlHandCommand(DoubleSupplier power) {
    return run(() -> manualHand(power));
  }

  public Command autoIntakeCommand() {
    return run(() -> autoIntake());
  }

  public Command primeEjectCommand() {
    return run(() -> primeEject());
  }

  public Command ejectCommand() {
    return run(() -> Eject());
  }

  // ********************* END OF COMMANDS ***************************/

  @Override
  public void periodic() {
    // The arm is perpendicular to the Upright shoulder.

    SmartDashboard.putBoolean("front beam break detects coral", frontBeamBreakDetectsCoral());
    SmartDashboard.putBoolean("back beam break detects coral", backBeamBreakDetectsCoral());
  }
}
