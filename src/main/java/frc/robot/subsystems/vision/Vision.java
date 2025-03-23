// Commando Robotics - FRC 5889
// Vision - Logic for getting information from the camera system

package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import java.util.List;
import java.util.Optional;
import org.littletonrobotics.junction.AutoLogOutput;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class Vision extends SubsystemBase {
  // Connections to the cameras
  private PhotonCamera frontLowCamera = new PhotonCamera("photonvision-frontlow");
  private PhotonCamera frontHighCamera = new PhotonCamera("photonvision-fronthigh");
  private PhotonCamera rearCamera = new PhotonCamera("photonvision-rear");

  // Last results from the cameras
  private PhotonPipelineResult lowLastResult = new PhotonPipelineResult();
  private PhotonPipelineResult highLastResult = new PhotonPipelineResult();
  private PhotonPipelineResult rearLastResult = new PhotonPipelineResult();

  // Locations of the cameras
  // The following distances are measured in meters.
  // x: Distance forward of center (towards front is positive, center is zero back of robot would be
  // negative).
  // y: Distance left of center (towards left is positive, center is zero, and right of center is
  // negative).
  // z: Distance above floor.
  // The following angles are measured in radians
  // roll: Rotated (usually not used)
  // tilt: Angle pointed up (positive) or down (negative)
  // yaw: Angle pointed left/clockwise (postive) or right/counter-clockwise (negative)
  // TODO: Tune the camera locations
  private Transform3d lowCameraLocation =
      new Transform3d(new Translation3d(.2, .2, .2), new Rotation3d(0, 0.79, -0.3));
  private Transform3d highCameraLocation =
      new Transform3d(new Translation3d(.2, 0, 1), new Rotation3d(0, 0.4, 0));
  private Transform3d rearCameraLocation =
      new Transform3d(new Translation3d(-.3, 0, 1), new Rotation3d(0, 0, 3.14));

  // Field April Tag locations
  AprilTagFieldLayout fieldLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

  // Tag we want to score at, or -1 for no sugestion (use best camera).
  int targetTag = -1;

  // Robot pose estimators
  PhotonPoseEstimator lowEstimator =
      new PhotonPoseEstimator(
          fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, lowCameraLocation);
  PhotonPoseEstimator highEstimator =
      new PhotonPoseEstimator(
          fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, highCameraLocation);
  PhotonPoseEstimator rearEstimator =
      new PhotonPoseEstimator(
          fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, rearCameraLocation);

  // Pose results from each camera
  Optional<EstimatedRobotPose> lowPose = Optional.empty();
  Optional<EstimatedRobotPose> highPose = Optional.empty();
  Optional<EstimatedRobotPose> rearPose = Optional.empty();

  // Best guess of where we are
  Pose3d bestPose = new Pose3d();

  // Estimated pose based on external sources (i.e. Odometry)
  Pose2d externalPose = new Pose2d();

  // Simulation
  private PhotonCameraSim lowSim;
  private PhotonCameraSim highSim;
  private PhotonCameraSim rearSim;
  private VisionSystemSim visionSim;

  // ********************* PUBLIC FUNCTIONS ***************************

  // If we are driving toward a specific tag, this will try to use a camera that sees it.
  // Use -1 to signify no specific tag (just use best camera).
  public void suggestTag(int number) {
    targetTag = number;
  }

  @AutoLogOutput(key = "Vision/Pose")
  public Pose2d getPose() {
    return bestPose.toPose2d();
  }

  // Provide best guess at pose from external source (i.e. Odometry)
  public void setPose(Pose2d pose) {
    externalPose = pose;
  }

  // ********************* END OF PUBLIC FUNCTIONS ***************************

  public Vision() {
    // Simulation stuff
    if (Robot.isSimulation()) {
      // Create the vision system simulation which handles cameras and targets on the field.
      visionSim = new VisionSystemSim("main");
      // Add all the AprilTags inside the tag layout as visible targets to this simulated field.
      visionSim.addAprilTags(fieldLayout);
      // Create simulated camera properties. These can be set to mimic your actual camera.
      var cameraProp = new SimCameraProperties();
      // Calibration angle is the Field-Of-View angle
      cameraProp.setCalibration(1280, 720, Rotation2d.fromDegrees(68.5));
      cameraProp.setCalibError(0.35, 0.10);
      // The following measurements are based on benchmarks on the PhotonVision site.
      cameraProp.setFPS(17);
      cameraProp.setAvgLatencyMs(39);
      cameraProp.setLatencyStdDevMs(15);

      // Create simulated cameras to update Network tables. All can use the same configuration.
      lowSim = new PhotonCameraSim(frontLowCamera, cameraProp);
      highSim = new PhotonCameraSim(frontHighCamera, cameraProp);
      rearSim = new PhotonCameraSim(rearCamera, cameraProp);
      // Provide location of the simulated cameras
      visionSim.addCamera(lowSim, lowCameraLocation);
      visionSim.addCamera(highSim, highCameraLocation);
      visionSim.addCamera(rearSim, rearCameraLocation);

      // Set up the initial position on the field
      externalPose = new Pose2d(0, 0, new Rotation2d(0));
    }
  }

  // Private function that checks to see if the specified April Tag was seen
  private boolean tagSeen(int tagNumber, PhotonPipelineResult result) {
    // Get all the targets from the pipeline
    List<PhotonTrackedTarget> targets = result.getTargets();

    // Go through each one to see if it matches the tage number.
    for (PhotonTrackedTarget t : targets) {
      if (t.getFiducialId() == tagNumber) {
        return true;
      }
    }
    // Was not found.
    return false;
  }

  private PhotonPipelineResult getLast(PhotonCamera c) {
    var results = c.getAllUnreadResults();
    // Check to see if there were any new results.
    // If we check to frequently (or another issue occurred) there will be no new results
    if (results.isEmpty()) {
      // Return an empty result.
      return new PhotonPipelineResult();
    }
    // Use the newest one in the list, which should be the last
    int size = results.size();
    // Indexes start at zero. So, if there were 3 items, the index of the last would be 2.
    int lastIndex = size - 1;
    PhotonPipelineResult lastResult = results.get(lastIndex);
    return lastResult;
  }

  // Estimtate the pose, choosing a camera that is aimed best at the reef
  // We may have to use other cameras if the best camera cannot see the tag (blocked)
  private Pose3d estimateReefScoring() {
    // Try to choose a camera that sees the tag
    if (targetTag != -1) {
      // The front low camera is the best for targeting reef tags.
      if (tagSeen(targetTag, lowLastResult)) {
        if (!lowPose.isEmpty()) {
          return lowPose.get().estimatedPose;
        }
      }
      // Might still be at Coral station, or driving backawrds. Use reverse camera.
      if (tagSeen(targetTag, rearLastResult)) {
        if (!rearPose.isEmpty()) {
          return rearPose.get().estimatedPose;
        }
      }
      // Unlikely that we would see a reef tag with the front high camera, but can try.
      if (tagSeen(targetTag, highLastResult)) {
        if (!highPose.isEmpty()) {
          return highPose.get().estimatedPose;
        }
      }
    }
    // No camera could see the desired tag (or we have no desired tag yet)
    // Maybe we are circling close to the reef, see if the front camera sees any tags.
    if (lowLastResult.hasTargets()) {
      if (!lowPose.isEmpty()) {
        return lowPose.get().estimatedPose;
      }
    }
    // See if rear camera sees any tags
    if (rearLastResult.hasTargets()) {
      if (!rearPose.isEmpty()) {
        return rearPose.get().estimatedPose;
      }
    }
    // See if high camera sees any tags
    if (highLastResult.hasTargets()) {
      if (!highPose.isEmpty()) {
        return highPose.get().estimatedPose;
      }
    }

    // We have no idea where we are.
    return new Pose3d();
  }

  // Estimate Pose based on camera order best for loading from human player station.
  private Pose3d estimateCoralStationLoading() {
    // Try to choose a camera that sees the tag
    if (targetTag != -1) {
      // The front high camera is the best for seeing the Coral Station tag.
      if (tagSeen(targetTag, highLastResult)) {
        if (!highPose.isEmpty()) {
          return highPose.get().estimatedPose;
        }
      }
      // The front low camera is the best if we are far from the Coral Station tag.
      if (tagSeen(targetTag, lowLastResult)) {
        if (!lowPose.isEmpty()) {
          return lowPose.get().estimatedPose;
        }
      }
      // We could be driving backwards
      if (tagSeen(targetTag, rearLastResult)) {
        if (!rearPose.isEmpty()) {
          return rearPose.get().estimatedPose;
        }
      }
    }
    // No camera could see the desired tag (or we have no desired tag yet)
    // Maybe we are circling close to the reef, see if the front camera sees any tags.
    if (lowLastResult.hasTargets()) {
      if (!lowPose.isEmpty()) {
        return lowPose.get().estimatedPose;
      }
    }
    // See if rear camera sees any tags
    if (rearLastResult.hasTargets()) {
      if (!rearPose.isEmpty()) {
        return rearPose.get().estimatedPose;
      }
    }
    // See if high camera sees any tags
    if (highLastResult.hasTargets()) {
      if (!highPose.isEmpty()) {
        return highPose.get().estimatedPose;
      }
    }

    // We have no idea where we are.
    return new Pose3d();
  }

  private Pose3d estimateProcessor() {
    // The camera selection for Processor is probably the same as Coral Station
    return estimateCoralStationLoading();
  }

  private Pose3d estimateBarge() {
    // Until we can tune for barge, just use Coral station order.
    return estimateCoralStationLoading();
  }

  // Estimate Pose based on the camera that sees the most tags.
  private Pose3d estimateMostTargets() {
    // Keep track of how many targets the camera with the best vision sees.
    int mostTargets = 0;
    int lowTargets = 0;
    if (!lowPose.isEmpty()) {
      lowTargets = lowLastResult.getTargets().size();
    }
    // For the moment, assume low camera sees the most.
    mostTargets = lowTargets;
    int highTargets = 0;
    if (!highPose.isEmpty()) {
      highTargets = highLastResult.getTargets().size();
    }
    if (highTargets > mostTargets) {
      mostTargets = highTargets;
    }
    int rearTargets = 0;
    if (!rearPose.isEmpty()) {
      rearTargets = rearLastResult.getTargets().size();
    }
    if (rearTargets > mostTargets) {
      mostTargets = rearTargets;
    }

    if (mostTargets == 0) {
      // No one saw anything. We have no idea where we are.
      return new Pose3d();
    }

    // Use the pose from whichever camera has the most targets
    if (lowTargets == mostTargets) {
      if (!lowPose.isEmpty()) {
        return lowPose.get().estimatedPose;
      }
    }
    if (highTargets == mostTargets) {
      if (!highPose.isEmpty()) {
        return highPose.get().estimatedPose;
      }
    }
    if (rearTargets == mostTargets) {
      if (!rearPose.isEmpty()) {
        return rearPose.get().estimatedPose;
      }
    }

    // It would be odd if we reach here, but would mean we have no idea where we are.
    return new Pose3d();
  }

  // Chooses the best pose based on the aprilTag
  private Pose3d estimateBestPose() {
    // Red Coral Station tags
    if (targetTag == 1 || targetTag == 2) {
      return estimateCoralStationLoading();
    }
    // Red Processor
    if (targetTag == 3) {
      return estimateProcessor();
    }
    // Red side of Barges
    if (targetTag == 4 || targetTag == 5) {
      return estimateBarge();
    }
    // Red Reef tags 6-11
    if (targetTag >= 6 && targetTag <= 11) {
      return estimateReefScoring();
    }
    // Blue Coral Station tags
    if (targetTag == 12 || targetTag == 13) {
      return estimateCoralStationLoading();
    }
    // Blue side of Barges
    if (targetTag == 14 || targetTag == 15) {
      return estimateBarge();
    }
    // Blue Processor
    if (targetTag == 16) {
      return estimateProcessor();
    }
    // Red Reef tags 17-22
    if (targetTag >= 17 && targetTag <= 22) {
      return estimateReefScoring();
    }

    // Unknown tag number
    return estimateMostTargets();
  }

  private void postPose(String prefix, Optional<EstimatedRobotPose> p) {
    double x = 0;
    double y = 0;
    double r = 0;
    if (!p.isEmpty()) {
      x = p.get().estimatedPose.getX();
      y = p.get().estimatedPose.getY();
      r = Rotation2d.fromRadians(p.get().estimatedPose.getRotation().getZ()).getDegrees();
    }
    SmartDashboard.putNumber(prefix + "X", x);
    SmartDashboard.putNumber(prefix + " Y", y);
    SmartDashboard.putNumber(prefix + "R (deg)", r);
  }

  @Override
  public void periodic() {
    // Update the latest result from each camera.
    var lowLastResult = getLast(frontLowCamera);
    var highLastResult = getLast(frontHighCamera);
    var rearLastResult = getLast(rearCamera);
    // Output basic information
    SmartDashboard.putBoolean("Low sees targets", lowLastResult.hasTargets());
    SmartDashboard.putBoolean("High sees targets", highLastResult.hasTargets());
    SmartDashboard.putBoolean("Rear sees targets", rearLastResult.hasTargets());

    // Estimate the pose based on each camera
    lowPose = lowEstimator.update(lowLastResult);
    highPose = lowEstimator.update(highLastResult);
    rearPose = lowEstimator.update(rearLastResult);
    postPose("low", lowPose);
    postPose("high", highPose);
    postPose("rear", rearPose);

    // Determine the best camera pose to use.
    bestPose = estimateBestPose();

    SmartDashboard.putNumber("targetTag", targetTag);
    SmartDashboard.putNumber("Best X", bestPose.getX());
    SmartDashboard.putNumber("Best Y", bestPose.getY());
    SmartDashboard.putNumber("Best Z (rad)", bestPose.getRotation().getAngle());
  }

  @Override
  public void simulationPeriodic() {
    // Update the simulated cameras to new Odometry location.
    visionSim.update(externalPose);
  }
}
