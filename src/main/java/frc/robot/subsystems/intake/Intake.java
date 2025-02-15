package frc.robot.subsystems.intake;

import java.util.function.DoubleSupplier;

import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

  private SparkMax armMotor;
  private SparkMax horizontalIntake;
  private SparkMax verticalIntake;
  private SparkMax indexerMotor;
  private DigitalInput intakeBeamBreak;
  private DigitalInput indexerBeamBreak;
  private Encoder armEncoder;
  private PIDController armPID;

  private static final double kP = 0.05; // Tune these values as needed
  private static final double kI = 0.0;
  private static final double kD = 0.0;

  private static final double horizontalIntakeSpeed = 0.5;
  private static final double verticalIntakeSpeed = 0.5;
  private static final double indexSpeed = 0.5;


  public Intake() {
    armMotor = new SparkMax(44, SparkMax.MotorType.kBrushless);
    horizontalIntake = new SparkMax(41, SparkMax.MotorType.kBrushless);
    verticalIntake = new SparkMax(42, SparkMax.MotorType.kBrushless); // car wash motor
    indexerMotor = new SparkMax(43, SparkMax.MotorType.kBrushless);
    intakeBeamBreak = new DigitalInput(6); // on the carwash portion of the robot
    indexerBeamBreak = new DigitalInput(7);
    armEncoder = new Encoder(4, 0); // TODO assign proper ports.
    armEncoder.setDistancePerPulse(360.0 / 8192); // sets the output of getDistance to degrees and puts the encoder in absolute mode


    armPID = new PIDController(kP, kI, kD);
    armPID.setTolerance(2.0); // Degrees of tolerance (adjust as needed)
  }

  public void resetArmEncoder() { // resets the arm encoder
    armEncoder.reset();
  }

  public void goToPosition(double targetPosition)  {  // targetPosition is in degrees
    double currentPosition = armEncoder.getDistance(); 
    double output = armPID.calculate(currentPosition, targetPosition);
    armMotor.set(output);
  }

  public boolean coralFullyCaptured() { // checks if coral is fully captured
    return intakeBeamBreak.get();
  }

  public boolean coralInIndexer() { // checks if coral is in the indexer
        return indexerBeamBreak.get();
  }

  public void stop() { // stops the arm motor
    armMotor.set(0);
    horizontalIntake.set(0);
    verticalIntake.set(0);
    indexerMotor.set(0);
  }


  // ********** INTAKE STATE METHODS ***********

  public void hungryIntake() { // runs the intake motors until the coral is fully captured
    if (!coralFullyCaptured()) {
      horizontalIntake.set(horizontalIntakeSpeed);
      verticalIntake.set(verticalIntakeSpeed);
    } else {
        horizontalIntake.set(0);
        verticalIntake.set(0);
    }
  }

  public void cleanOut() { // runs the horizontal intake motor in reverse to flush out the coral
    horizontalIntake.set(-horizontalIntakeSpeed);
  }

  public void digest() { // runs the indexer motors until the coral exits the indexer
    if (coralInIndexer()) {
      indexerMotor.set(indexSpeed);
    } else {
      verticalIntake.set(verticalIntakeSpeed);
      indexerMotor.set(0);

    }
  }

  public void full() { // stops all motors
    horizontalIntake.set(0);
    verticalIntake.set(0);
    indexerMotor.set(0);
  }

  // ************END INTAKE STATE METHODS*******************




  //********* COMMANDS ************

    public Command stopCommand() { // stops all Arm/Intake/Indexer motors
        return run(() -> stop());
    }

    public Command goToPositionCommand(double targetPosition) { // runs the arm motor until the target position is reached
        return run(() -> goToPosition(targetPosition));
    }

    public Command hungryIntakeCommand() { // runs the intake motors until the coral is fully captured
        return run(() -> hungryIntake());
    }

    public Command cleanOutCommand() { // runs the horizontal intake motor in reverse to flush out the coral
        return run(() -> cleanOut());
    }

    public Command digestCommand() { // runs the indexer motors until the coral exits the indexer
        return run(() -> digest());
    }

    public Command fullCommand() { // stops all motors associated with Intake and indexer
        return run(() -> full());
    }


    // Use this if you want to track an joystick (XBox or flightstick). 
    public Command trackCommand(DoubleSupplier value) {
        return run(
            () -> {
                goToPosition(value.getAsDouble() * 360);
            }
        );
    }

    //********* END COMMANDS ************



   @Override
    public void periodic() {
        // The arm is perpendicular to the Upright shoulder.
        SmartDashboard.putBoolean("intake detects note", coralFullyCaptured());
        SmartDashboard.putBoolean("indexer detects note", coralInIndexer());
        

    }










}


