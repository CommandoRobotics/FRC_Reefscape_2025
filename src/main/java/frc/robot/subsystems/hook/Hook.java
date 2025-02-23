package frc.robot.subsystems.hook;

import org.littletonrobotics.junction.AutoLogOutput;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

enum hookPositions {
    stowed, // Starting position.
    grab, // Grab from the reef.
    barge, // Shooting into the barge.
    protect, // Bring algae into robot so it doesn't stick out front.
    avoid, // Stick the hook out the front so it can't conflict with the climber.
}


public class Hook extends SubsystemBase {
    private SparkMax grabberMotor;
    private boolean autoGrab;
    private boolean autoYeet;
    private final int touchSensorPort = 0;
    private DigitalInput touchSensor;
    double grab_motor_power = 0.5; // TODO: Tune this value
    
    private Rotation2d targetAngle;
    

    // Shoulder Motor
  final int shoulderMotorCANID = 61;
  private SparkMax shoulderMotor; // Motor for rotating the hand up and down.
  private SparkMaxConfig shoulderMotorConfig;
  private SparkClosedLoopController shoulderClosedLoopController;

  // Encoder is plugged directly into Shoulder motor
  // Allows access to the attached Encoder
  private AbsoluteEncoder shoulderEncoder;
  // Angle (rotations) the HexboreEncoder reads when vertical.
  private final Rotation2d shoulderOffsetAngleInRotations;
  // Variable to store the last read rotation of the motor.
  @AutoLogOutput private Rotation2d actualAngle;
  // Don't allow the hook to rotate up past this, or it will break (wires might yank).
  private final Rotation2d maxAngle = Rotation2d.fromDegrees(180);
  // Don't allow the hook to rotate down past this, or it will break (wires might yank).
  private final Rotation2d minAngle = Rotation2d.fromDegrees(-90);

    

    //Constructor
    public Hook() {
    grabberMotor = new SparkMax(62, MotorType.kBrushless);
    autoGrab = false;
    autoYeet = true;
    touchSensor = new DigitalInput(touchSensorPort);

    targetAngle = Rotation2d.fromDegrees(0);
    shoulderOffsetAngleInRotations = Rotation2d.fromDegrees(0); // TODO: Tune this value
    shoulderMotor = new SparkMax(shoulderMotorCANID, MotorType.kBrushless);
    shoulderClosedLoopController = shoulderMotor.getClosedLoopController();
    shoulderEncoder = shoulderMotor.getAbsoluteEncoder();
    // Configure the shoulder motor
    shoulderMotorConfig = new SparkMaxConfig();
    shoulderMotorConfig
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
    shoulderMotor.configure(
        shoulderMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

}

public void setTargetPosition(hookPositions position) {
    switch (position) {
        case stowed:
            targetAngle = Rotation2d.fromDegrees(0);
            break;
        case grab:
            targetAngle = Rotation2d.fromDegrees(45);
            break;
        case barge:
            targetAngle = Rotation2d.fromDegrees(180);
            break;
        case protect:
            targetAngle = Rotation2d.fromDegrees(-45);
            break;
        case avoid:
            targetAngle = Rotation2d.fromDegrees(10);
            break;
        default:
            targetAngle = Rotation2d.fromDegrees(0);
            break;
    }
}
// Function to run the (intake) grabber motor
private void grab() {
    grabberMotor.set(grab_motor_power);
}





// Function to reverse (yeet) grabber motor
private void yeet() {
    grabberMotor.set(-0.5);
}

// Function to know if the hook has algae
public boolean hasAlgae() {
    return touchSensor.get();
}

// Command to set auto grab
public void autoGrab() {
    if (autoGrab) {
        grab();
    }
}

public void autoYeet() {
    if (autoYeet) {
        yeet();
    }
}


@Override
public void periodic() {
    // Need to adjust the angle to account for the sensor offset for the hook
   Rotation2d adjustedTarget = targetAngle.plus(shoulderOffsetAngleInRotations);
    shoulderClosedLoopController.setReference(adjustedTarget.getRotations(), ControlType.kPosition, ClosedLoopSlot.kSlot0);
    SmartDashboard.putNumber("Target Position (Degrees)", targetAngle.getDegrees());


    if(autoGrab) {
        // Check if button is pressed
        if(touchSensor.get()) {
            // If button is pressed, stop the motor
            grabberMotor.set(0);
        } else {
            // If button is not pressed, run the motor
            grabberMotor.set(grab_motor_power);
        }
    }

    if(autoYeet) {
        // Check if button is pressed
        if(touchSensor.get()) {
            // If button is pressed, stop the motor
            grabberMotor.set(-0.5);
        } else {
            // If button is not pressed, run the motor
            grabberMotor.set(0.0);
        }
    }
}

//****** COMMANDS ********* */
public Command setStowedPositionCommand() {
    return run(() -> setTargetPosition(hookPositions.stowed));
}


public Command setGrabPositionCommand() {
    return run(() -> setTargetPosition(hookPositions.grab));
}


public Command setBargePositionCommand() {
    return run(() -> setTargetPosition(hookPositions.barge));
}


public Command setProtectPositionCommand() {
    return run(()-> setTargetPosition(hookPositions.protect));
}


public Command setAvoidPositionCommand() {
    return run(()-> setTargetPosition(hookPositions.avoid));
}


public Command autoGrabCommand() {
    return run(() -> autoGrab());
}


public Command autoYeetCommand() {
    return run(()-> autoYeet());
}
}

