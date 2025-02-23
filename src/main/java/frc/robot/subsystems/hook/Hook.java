package frc.robot.subsystems.hook;

import org.littletonrobotics.junction.AutoLogOutput;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DigitalInput;
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
    private final int touchSensorPort = 0;
    private DigitalInput touchSensor;




    private Rotation2d targetAngle;
    // TODO: Rename wrist to schoulder
    // Wrist Motor
  final int shoulderMotorCANID = 61;
  private SparkMax shoulderMotor; // Motor for rotating the hand up and down.
  private SparkMaxConfig shoulderMotorConfig;
  private SparkClosedLoopController shoulderClosedLoopController;
  private SparkPIDController shoulderPIDController;
  // Encoder is plugged directly into Wrist motor
  // Allows access to the attached Encoder
  private AbsoluteEncoder shoulderEncoder;
  // Angle (rotations) the HexboreEncoder reads when vertical.
  private final double shoulderOffsetAngleInRotations = 0.0;
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
    touchSensor = new DigitalInput(touchSensorPort);

    targetAngle = Rotation2d.fromDegrees(0);
    shoulderMotor = new SparkMax(shoulderMotorCANID, MotorType.kBrushless);
    shoulderClosedLoopController = shoulderMotor.getClosedLoopController();
    shoulderEncoder = shoulderMotor.getAbsoluteEncoder();
    // Configure the wrist motor
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

// TODO: Create function to run (intake) the grabber motor
private void grab() {
    double grab_motor_power = 0.5; // TODO: Tune this value
    grabberMotor.set(grab_motor_power);
}

// TODO: Create function to reverse (yeet) the grabber motor

// TODO: Create a command to set auto grab

@Override
public void periodic() {
    shoulderClosedLoopController.setReference(targetAngle.getRotations(), ControlType.kPosition, ClosedLoopSlot.kSlot0);

    if (autoGrab) {
        // Check if button is pressed
        if (touchSensor.get()) {
            // if so, run the motors the ball
        } else {
            // else stop the motor
        }
    }
}

}

