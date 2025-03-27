// Commando Robotics - FRC 5889
// Autopilot - Logic for determining where the robot should go

package frc.robot.subsystems.autopilot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;

public class Autopilot extends SubsystemBase {
  // Store the best guess of where we are here.
  private Pose2d currentPose = new Pose2d();
  // Location of the scoring element and angle we should be at to face it
  private Pose2d targetPose = new Pose2d();
  // For debugging, store the point on the field we want to go to and desired angle there
  private Pose2d desiredPose = new Pose2d();
  // For debuggnig, store the offset we think we are from the target
  private Transform2d drivingDirection = new Transform2d();
  // Store the translation we want for the robot (x, y, rotation)
  private Pose2d driveControls = new Pose2d();
  // Simulated location of the robot, as if this module were driving it.
  private Pose2d simulatedPose = new Pose2d();

  // TODO: Tune these limits
  private double max_translational_stick =
      0.25; // Max power auto is allowed drive forward/back/left/right
  private Rotation2d max_rotational_stick =
      Rotation2d.fromDegrees(45); // Max power auto is allowed to rotate the robot

  private double min_translational_stick = 0.01; // Similiar to a joystick deadband
  private Rotation2d min_rotational_stick =
      Rotation2d.fromDegrees(1); // Similiar to a joystick deadband

  public void setCurrentPose(Pose2d robot) {
    currentPose = robot;
  }

  public void setElementPose(Pose2d target) {
    targetPose = target;
  }

  // This returns the location the Autopilot thinks the robot should go to
  @AutoLogOutput(key = "Autopilot/TargetPose")
  public Pose2d getTargetPose() {
    return desiredPose;
  }
  // This returns the location the Autopilot thinks the robot should go to
  @AutoLogOutput(key = "Autopilot/CurrentPose")
  public Pose2d getCurrentPose() {
    return currentPose;
  }

  // This returns the location the Autopilot thinks the robot should go to
  @AutoLogOutput(key = "Autopilot/SimulatedPose")
  public Pose2d getSimulatedPose() {
    return simulatedPose;
  }

  // This returns the location the Autopilot thinks the robot should go to
  @AutoLogOutput(key = "Autopilot/Transform")
  public Transform2d getTranslation() {
    return drivingDirection;
  }

  @AutoLogOutput(key = "Autopilot/Drive")
  public Pose2d getDrivePose() {
    return driveControls;
  }

  @AutoLogOutput(key = "Autopilot/DriveX")
  public double drivePowerX() {
    return driveControls.getX();
  }

  @AutoLogOutput(key = "Autopilot/DriveY")
  public double drivePowerY() {
    return driveControls.getY();
  }

  @AutoLogOutput(key = "Autopilot/DriveR")
  public Rotation2d drivePowerR() {
    return driveControls.getRotation();
  }

  // Translates the pose of the target to where the center of the robot should be.
  // Swerve code is relative to the center of the robot.
  private Pose2d getRobotPoseFromElementPose(Pose2d elementPose) {
    // The bumpers shift the robot back (180 degrees) 3.25 inches
    // 3.25 inches = 0.083 meters
    final double bumperWidthMeters = 0.083;
    Translation2d bumperTranslation =
        new Translation2d(bumperWidthMeters, Rotation2d.fromDegrees(180));
    Transform2d bumperTransform = new Transform2d(bumperTranslation, Rotation2d.fromDegrees(0));
    Pose2d bumperPose = elementPose.transformBy(bumperTransform);

    // We want the center of the hand to align with the target element
    // The Hand is 4 inches to the left from the front center of the robot.
    // So shift the location by 4 inches to the right (-90 degrees) to get where the robot center
    // should be.
    final double handOffsetMeters = 0.1; // 4 inches = 0.1 meters
    Translation2d handTranslation =
        new Translation2d(handOffsetMeters, Rotation2d.fromDegrees(-90));
    Transform2d handTransform = new Transform2d(handTranslation, Rotation2d.fromDegrees(0));
    Pose2d frontMiddlePose = bumperPose.transformBy(handTransform);

    // The robot is 30 inches long, so move 15 inches back to get where the center is
    final double centerOffsetMeters = 0.38; // 15 inches = 0.38 meters
    Translation2d centerTranslation =
        new Translation2d(centerOffsetMeters, Rotation2d.fromDegrees(180));
    Transform2d centerTransform = new Transform2d(centerTranslation, Rotation2d.fromDegrees(0));
    Pose2d centerPose = frontMiddlePose.transformBy(centerTransform);

    return centerPose;
  }

  // Value beteen -1 to +1
  private double toDrivePower(double distanceInMeters) {
    double absoluteDistance = Math.abs(distanceInMeters);
    double direction = 1;
    if (distanceInMeters < 0) {
      direction = -1;
    }
    double drivePower = max_translational_stick;
    if (absoluteDistance < 0.025) {
      // If we are closer than 1 inch, stop
      drivePower = 0;
    } else if (absoluteDistance < 0.5) {
      // If we are in a few feet, drive slow
      // TODO: Change this to a range of values
      drivePower = 0.05;
    } else if (absoluteDistance < 1) {
      // If we are in a few feet, drive slow
      // TODO: Change this to a range of values
      drivePower = 0.1;
    }

    return direction * drivePower;
  }

  // Returns an angle betwen -360 to 360 degrees
  private double normalizeAngle(double angleInDegrees) {
    if (angleInDegrees > 0) {
      while (angleInDegrees > 360) {
        angleInDegrees -= 360;
      }
      return angleInDegrees;
    } else {
      // Negative angle
      while (angleInDegrees < -360) {
        angleInDegrees += 360;
      }
      return angleInDegrees;
    }
  }

  private Rotation2d toRotationPower(Rotation2d angle) {
    double angleInDegrees = normalizeAngle(angle.getDegrees());
    double direction = 1;
    if (angleInDegrees < 0) {
      direction = -1;
    }
    double absoluteAngle = Math.abs(angleInDegrees);
    double anglePower = max_rotational_stick.getDegrees();
    // TODO: Tune this
    if (absoluteAngle < 2) {
      // Within a couple of degrees, do not rotate
      anglePower = 0;
    } else if (absoluteAngle < 20) {
      anglePower = 10;
    }
    return Rotation2d.fromDegrees(direction * anglePower);
  }

  @Override
  public void periodic() {
    desiredPose = getRobotPoseFromElementPose(targetPose);
    double deltaX = desiredPose.getX() - currentPose.getX();
    double driveX = toDrivePower(deltaX);
    double deltaY = desiredPose.getY() - currentPose.getY();
    double driveY = toDrivePower(deltaY);
    Rotation2d deltaR = desiredPose.getRotation().minus(currentPose.getRotation());
    Rotation2d driveR = toRotationPower(deltaR);
    drivingDirection = new Transform2d(deltaX, deltaY, deltaR);
    driveControls = new Pose2d(driveX, driveY, driveR);
  }

  @Override
  public void simulationPeriodic() {
    double simulatedDriveDistance = 0.01;
    double simulatedRotationDegrees = 0.01;

    double x = simulatedPose.getX() + drivePowerX() * simulatedDriveDistance;
    double y = simulatedPose.getY() + drivePowerY() * simulatedDriveDistance;
    Rotation2d r =
        simulatedPose
            .getRotation()
            .plus(Rotation2d.fromDegrees(drivePowerR().getDegrees() * simulatedRotationDegrees));
    simulatedPose = new Pose2d(x, y, r);
    currentPose = simulatedPose;
  }
}
