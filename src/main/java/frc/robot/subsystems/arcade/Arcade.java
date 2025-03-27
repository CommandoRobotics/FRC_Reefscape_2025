// Commando Robotics - FRC 5889
// Arcade - Logic for the custom game controller

package frc.robot.subsystems.arcade;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Optional;
import org.littletonrobotics.junction.AutoLogOutput;

// Arcade stores the last commanded targets by the arm operator, for autopilot reference.
public class Arcade extends SubsystemBase {
  // ********************* COMMANDS ***************************
  public Command climbButtonCommand() {
    return run(() -> setClimb());
  }

  public Command leftProcessorButtonCommand() {
    return run(() -> setProcessor(targetLocation.leftProcessor));
  }

  public Command rightProcessorButtonCommand() {
    return run(() -> setProcessor(targetLocation.rightProcessor));
  }

  public Command bargeButtonCommand() {
    return run(() -> setBarge());
  }

  public Command leftCoralStationButtonCommand() {
    return run(() -> setCoralStation(targetLocation.leftCoralStation));
  }

  public Command rightCoralStationButtonCommand() {
    return run(() -> setCoralStation(targetLocation.rightCoralStation));
  }

  public Command l1ButtonCommand() {
    return run(() -> setLevel(targetHeight.bottom));
  }

  public Command l2ButtonCommand() {
    return run(() -> setLevel(targetHeight.L2));
  }

  public Command l3ButtonCommand() {
    return run(() -> setLevel(targetHeight.L3));
  }

  public Command l4ButtonCommand() {
    return run(() -> setLevel(targetHeight.L4));
  }

  public Command reefAButtonCommand() {
    return run(() -> setReef(targetLocation.reefA));
  }

  public Command reefBButtonCommand() {
    return run(() -> setReef(targetLocation.reefB));
  }

  public Command reefCButtonCommand() {
    return run(() -> setReef(targetLocation.reefC));
  }

  public Command reefDButtonCommand() {
    return run(() -> setReef(targetLocation.reefD));
  }

  public Command reefEButtonCommand() {
    return run(() -> setReef(targetLocation.reefE));
  }

  public Command reefFButtonCommand() {
    return run(() -> setReef(targetLocation.reefF));
  }

  public Command reefGButtonCommand() {
    return run(() -> setReef(targetLocation.reefG));
  }

  public Command reefHButtonCommand() {
    return run(() -> setReef(targetLocation.reefH));
  }

  public Command reefIButtonCommand() {
    return run(() -> setReef(targetLocation.reefI));
  }

  public Command reefJButtonCommand() {
    return run(() -> setReef(targetLocation.reefJ));
  }

  public Command reefKButtonCommand() {
    return run(() -> setReef(targetLocation.reefK));
  }

  public Command reefLButtonCommand() {
    return run(() -> setReef(targetLocation.reefL));
  }

  // ********************* END OF COMMANDS ********************

  // This returns the location of the Field Element we want to go to
  // and the rotation our robot should be pointed at.
  @AutoLogOutput(key = "Arcade/Pose")
  public Pose2d getTargetPose() {
    return determineElementPose();
  }

  // Which set of rules the autopilot should operate under
  private enum operatingMode {
    placeCoralOnReef,
    scoreAlgaeInProcessor,
    tossAlgaeInBarge,
    recieveCoralFromHumanPlayer,
    climb,
    unknown
  }

  // Which Field Element we are targeting
  private enum targetLocation {
    reefA,
    reefB,
    reefC,
    reefD,
    reefE,
    reefF,
    reefG,
    reefH,
    reefI,
    reefJ,
    reefK,
    reefL,
    leftCoralStation,
    rightCoralStation,
    leftProcessor,
    rightProcessor,
    barge,
    cage,
    unknown
  }

  // Which level the Elevator should go to
  private enum targetHeight {
    bottom,
    L2,
    L3,
    L4,
    barge,
    unknown
  }

  private operatingMode currentMode = operatingMode.unknown;
  private targetLocation currentTargetLocation = targetLocation.unknown;
  private targetHeight currentTargetHeight = targetHeight.unknown;

  // Constructor
  public Arcade() {}

  public void set(operatingMode m, targetLocation l, targetHeight h) {
    currentMode = m;
    currentTargetLocation = l;
    currentTargetHeight = h;
  }

  private void setClimb() {
    set(operatingMode.climb, targetLocation.cage, targetHeight.bottom);
  }

  private void setProcessor(targetLocation l) {
    set(operatingMode.scoreAlgaeInProcessor, l, targetHeight.bottom);
  }

  private void setBarge() {
    set(operatingMode.tossAlgaeInBarge, targetLocation.barge, targetHeight.barge);
  }

  private void setCoralStation(targetLocation l) {
    set(operatingMode.recieveCoralFromHumanPlayer, l, targetHeight.bottom);
  }

  private void setReef(targetLocation l) {
    currentMode = operatingMode.placeCoralOnReef;
    currentTargetLocation = l;
    // We do not know the height, that will be set by a level button.
  }

  private void setLevel(targetHeight h) {
    // We do not know the location or mode. That is set by another button.
    currentTargetHeight = h;
  }

  // Returns a Pose2d indicating a point at the center of the Tag, but facing the tag
  // Values use WPILib coordinates (Blue's right corner is origin), distances in meters.
  // Positions are based on Photon Vision Welded map, but adjusted angles.
  private Pose2d determineTagPose(int tagNumber) {
    switch (tagNumber) {
        // Red Side
      case 1:
        return new Pose2d(
            16.697198, 0.65532, Rotation2d.fromDegrees(-54)); // Red's Left Coral Station
      case 2:
        return new Pose2d(
            16.697198, 7.3964799999999995, Rotation2d.fromDegrees(54)); // Red's Right Coral Station
      case 3:
        return new Pose2d(
            11.560809999999998, 8, Rotation2d.fromDegrees(90)); // Red Side Processor Station
      case 4:
        return new Pose2d(
            9.276079999999999, 6.137656, Rotation2d.fromDegrees(180)); // Red's Right Barge (Blue)
      case 5:
        return new Pose2d(
            9.276079999999999, 1.914906, Rotation2d.fromDegrees(180)); // Red's Left Barge (Red)
      case 6:
        return new Pose2d(
            13.474446, 3.3063179999999996, Rotation2d.fromDegrees(120)); // Red KL Reef
      case 7:
        return new Pose2d(13.890498, 4.0259, Rotation2d.fromDegrees(180)); // Red AB Reef
      case 8:
        return new Pose2d(13.474446, 4.745482, Rotation2d.fromDegrees(-120)); // Red CD Reef
      case 9:
        return new Pose2d(12.643358, 4.745482, Rotation2d.fromDegrees(-60)); // Red EF Reef
      case 10:
        return new Pose2d(12.227305999999999, 4.0259, Rotation2d.fromDegrees(0)); // Red GH Reef
      case 11:
        return new Pose2d(12.643358, 3.3063179999999996, Rotation2d.fromDegrees(60)); // Red IJ Reef
        // Blue Side
      case 12:
        return new Pose2d(
            0.851154, 0.65532, Rotation2d.fromDegrees(-126)); // Blue's Right Coral Station
      case 13:
        return new Pose2d(
            0.851154, 7.3964799999999995, Rotation2d.fromDegrees(126)); // Blue's Left Coral Station
      case 14:
        return new Pose2d(
            8.272272, 6.137656, Rotation2d.fromDegrees(0)); // Blue's Left Barge (Blue)
      case 15:
        return new Pose2d(
            8.272272, 1.914906, Rotation2d.fromDegrees(0)); // Blue's Right Barge (Red)
      case 16:
        return new Pose2d(
            5.9875419999999995, 0, Rotation2d.fromDegrees(-90)); // Blue Side Processor
      case 17:
        return new Pose2d(
            4.073905999999999, 3.3063179999999996, Rotation2d.fromDegrees(60)); // Blue CD Reef
      case 18:
        return new Pose2d(3.6576, 4.0259, Rotation2d.fromDegrees(0)); // Blue AB Reef
      case 19:
        return new Pose2d(4.073905999999999, 4.745482, Rotation2d.fromDegrees(-60)); // Blue KL Reef
      case 20:
        return new Pose2d(
            4.904739999999999, 4.745482, Rotation2d.fromDegrees(-120)); // Blue IJ Reef
      case 21:
        return new Pose2d(5.321046, 4.0259, Rotation2d.fromDegrees(180)); // Blue GH Reef
      case 22:
        return new Pose2d(
            4.904739999999999, 3.3063179999999996, Rotation2d.fromDegrees(120)); // Blue EF Reef
      default:
        return new Pose2d();
    }
  }

  private Pose2d translateReefPose(boolean left, Pose2d tagPose) {
    // The reef poles are 1'1" apart, center-to-center (see game manual 5.3)
    // 1'1" = 0.33 meters
    // The April Tag's center is directly centered between the reef poles
    // 0.33m / 2 = 0.165m from the center of the tag to the center of a pole
    final double tagCenterToPoleCenterInMeters = 0.165;
    // The direction to translate is either to the left (90 degrees) or right (-90 degrees)
    Rotation2d translationAngle = Rotation2d.fromDegrees(90);
    if (!left) {
      // Go the other way
      translationAngle = Rotation2d.fromDegrees(-90);
    }
    // Move the distance by the same
    Transform2d t =
        new Transform2d(
            new Translation2d(tagCenterToPoleCenterInMeters, translationAngle),
            Rotation2d.fromDegrees(0));
    return tagPose.transformBy(t);
  }

  // Provides the Pose that the scoring mechanism should be at score on that object
  private Pose2d determineElementPose() {
    boolean blue = true;
    Optional<Alliance> a = DriverStation.getAlliance();
    if (a.isPresent()) {
      if (a.get() == Alliance.Red) {
        blue = false;
      }
    }

    switch (currentTargetLocation) {
      case reefA:
        if (blue) {
          return translateReefPose(true, determineTagPose(18));
        } else {
          return translateReefPose(true, determineTagPose(7));
        }
      case reefB:
        if (blue) {
          return translateReefPose(false, determineTagPose(18));
        } else {
          return translateReefPose(false, determineTagPose(7));
        }
      case reefC:
        if (blue) {
          return translateReefPose(true, determineTagPose(17));
        } else {
          return translateReefPose(true, determineTagPose(8));
        }
      case reefD:
        if (blue) {
          return translateReefPose(false, determineTagPose(17));
        } else {
          return translateReefPose(false, determineTagPose(8));
        }
      case reefE:
        if (blue) {
          return translateReefPose(true, determineTagPose(22));
        } else {
          return translateReefPose(true, determineTagPose(9));
        }
      case reefF:
        if (blue) {
          return translateReefPose(false, determineTagPose(22));
        } else {
          return translateReefPose(false, determineTagPose(9));
        }
      case reefG:
        if (blue) {
          return translateReefPose(true, determineTagPose(21));
        } else {
          return translateReefPose(true, determineTagPose(10));
        }
      case reefH:
        if (blue) {
          return translateReefPose(false, determineTagPose(21));
        } else {
          return translateReefPose(false, determineTagPose(10));
        }
      case reefI:
        if (blue) {
          return translateReefPose(true, determineTagPose(20));
        } else {
          return translateReefPose(true, determineTagPose(10));
        }
      case reefJ:
        if (blue) {
          return translateReefPose(false, determineTagPose(20));
        } else {
          return translateReefPose(false, determineTagPose(10));
        }
      case reefK:
        if (blue) {
          return translateReefPose(true, determineTagPose(19));
        } else {
          return translateReefPose(true, determineTagPose(6));
        }
      case reefL:
        if (blue) {
          return translateReefPose(false, determineTagPose(19));
        } else {
          return translateReefPose(false, determineTagPose(6));
        }
      case leftCoralStation:
        if (blue) {
          return determineTagPose(13);
        } else {
          return determineTagPose(1);
        }
      case rightCoralStation:
        if (blue) {
          return determineTagPose(12);
        } else {
          return determineTagPose(2);
        }
      case leftProcessor:
        if (blue) {
          return determineTagPose(3);
        } else {
          return determineTagPose(16);
        }
      case rightProcessor:
        if (blue) {
          return determineTagPose(16);
        } else {
          return determineTagPose(3);
        }
      case barge:
        if (blue) {
          return determineTagPose(14);
        } else {
          return determineTagPose(5);
        }
      case cage:
        if (blue) {
          return determineTagPose(14);
        } else {
          return determineTagPose(5);
        }
      default:
        return new Pose2d();
    }
  }

  @Override
  public void periodic() {
    // Post values to dashboard for debugging.
    SmartDashboard.putString("Arcade Mode", currentMode.toString());
    SmartDashboard.putString("Arcade Target Location", currentTargetLocation.toString());
    Pose2d targetPose = determineElementPose();
    SmartDashboard.putNumber("Arcade Target X", targetPose.getX());
    SmartDashboard.putNumber("Arcade Target Y", targetPose.getY());
    SmartDashboard.putNumber("Arcade Target R (deg)", targetPose.getRotation().getDegrees());
    SmartDashboard.putString("Arcade Target Height", currentTargetHeight.toString());
  }

  @Override
  public void simulationPeriodic() {}
}
