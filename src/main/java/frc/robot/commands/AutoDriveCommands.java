package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.autopilot.Autopilot;
import frc.robot.subsystems.drive.Drive;

public class AutoDriveCommands {

  private AutoDriveCommands() {}

  public static Command driveUsingAutoPilot(Drive drive, Autopilot autopilot) {

    // Construct command
    return DriveCommands.joystickDriveAtAngle(
        drive,
        () -> {
          return autopilot.drivePowerX();
        },
        () -> {
          return autopilot.drivePowerY();
        },
        () -> {
          return autopilot.drivePowerR();
        });
  }
}
