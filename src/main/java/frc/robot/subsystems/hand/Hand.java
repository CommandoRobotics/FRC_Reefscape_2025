
package frc.robot.subsystems.hand;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Hand extends SubsystemBase {

  private SparkMax wristMotor;
  private SparkMax rollerMotor;
  private DigitalInput handBeamBreak;
  private Encoder wristEncoder;
  private PIDController wristPID;

  private static final double kP = 0.05; // Tune these values as needed
  private static final double kI = 0.0;
  private static final double kD = 0.0;

  private static final double rollerSpeed = 0.5; // Speed of the roller motor
  private static final double intakeHumanPlayerPositionDegrees = 95; // position in degrees the hand needs to be to intake from human player // TODO tune this value
  private static final double intakeFromIndexerPositionDegrees = 10; // position in degrees the hand needs to be to intake from indexer // TODO tune this value
  private static final double L1PositionDegrees = 0; // position in degrees the hand needs to be to score at L1 // TODO tune this value
  private static final double L2PositionDegrees = 45; // position in degrees the hand needs to be to score at L2 // TODO tune this value
  private static final double L3PositionDegrees = 45; // position in degrees the hand needs to be to score at L2 // TODO tune this value
  private static final double L4PositionDegrees = 90; // position in degrees the hand needs to be to score at L2 // TODO tune this value



  public Hand() {
    wristMotor = new SparkMax(51, SparkMax.MotorType.kBrushless);
    rollerMotor = new SparkMax(53, SparkMax.MotorType.kBrushless);
    handBeamBreak = new DigitalInput(9); // TODO change port number
    wristEncoder = new Encoder(4, 0); // TODO assign proper ports.
    wristEncoder.setDistancePerPulse(360.0 / 8192); // sets the output of getDistance to degrees and puts the encoder in absolute mode


    wristPID = new PIDController(kP, kI, kD);
    wristPID.setTolerance(2.0); // Degrees of tolerance (adjust as needed)
  }

  public void resetWristEncoder() { // resets the wrist encoder
    wristEncoder.reset();
  }

  public void goToPosition(double targetPosition)  {  // targetPosition is in degrees
    double currentPosition = wristEncoder.getDistance(); 
    double output = wristPID.calculate(currentPosition, targetPosition);
    wristMotor.set(output);
  }

  public boolean coralInHand() { // checks if coral is fully captured
    return handBeamBreak.get();
  }


  public void stop() { // stops the hand subsystem
    wristMotor.set(0);
    rollerMotor.set(0);

  }

  public void autoIntakeHumanPlayer() { // automatically intakes coral from human player
    goToPosition(intakeHumanPlayerPositionDegrees);
    if (!coralInHand()) {
      rollerMotor.set(rollerSpeed); 
    } else {
      rollerMotor.set(0);
    }
  }

  
  public void autoIntakeIndexer() { // automatically intakes coral from indexer
    goToPosition(intakeFromIndexerPositionDegrees);
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
      goToPosition(L1PositionDegrees); 
    } else if (Level == "L2") {
      goToPosition(L2PositionDegrees);  
    } else if (Level == "L3") {
      goToPosition(L3PositionDegrees); 
    } else if (Level == "L4") {
      goToPosition(L4PositionDegrees); 
    } else {
      goToPosition(0);
    }
  }

  //********************* COMMANDS ***************************/

  public Command stopCommand() { // stops all hand subsystem motors
        return run(() -> stop());
  }

  public Command goToPositionCommand(double targetPosition) { // runs the wrist motor until the target position is reached
    return run(() -> goToPosition(targetPosition));
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

  public Command goToScorePostionCommand(String Level) { // scores coral TAKES INPUT of L1, L2, L3, or L4
    return run(() -> goToScorePostion(Level));
  }

  //********************* END OF COMMANDS ***************************/






   @Override
    public void periodic() {
        // The arm is perpendicular to the Upright shoulder.
        SmartDashboard.putBoolean("hand detects coral", coralInHand());
        

    }




}


