// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.arcade.Arcade;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIONavX;
import frc.robot.subsystems.drive.ModuleIOSpark;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.hand.Hand;
import frc.robot.subsystems.hook.Hook;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
// import frc.robot.commands.ScoreAndReplaceCommand;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final Arcade arcade = new Arcade();

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);
  private final CommandXboxController armController = new CommandXboxController(1);
  private final CommandGenericHID reefPad = new CommandGenericHID(2);
  private final CommandGenericHID otherPad = new CommandGenericHID(3);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  private final Elevator elevator;

  private final Hand hand;

  private final Hook hook;

  private final Climb climb;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    // Real robot, instantiate hardware IO implementations
    drive =
        new Drive(
            new GyroIONavX(),
            new ModuleIOSpark(0),
            new ModuleIOSpark(1),
            new ModuleIOSpark(2),
            new ModuleIOSpark(3));

    /* case SIM:
      // Sim robot, instantiate physics sim IO implementations
      drive =
          new Drive(
              new GyroIO() {},
              new ModuleIOSim(),
              new ModuleIOSim(),
              new ModuleIOSim(),
              new ModuleIOSim());
      break;

    default:
      // Replayed robot, disable IO implementations
      drive =
          new Drive(
              new GyroIO() {},
              new ModuleIO() {},
              new ModuleIO() {},
              new ModuleIO() {},
              new ModuleIO() {});
      break; */

    elevator = new Elevator();
    hand = new Hand();
    hook = new Hook();
    climb = new Climb();

    // Set up auto routines
    autoChooser =
        new LoggedDashboardChooser<>(
            "Auto Choices (NOTE: the side of the field that has your color of cages is the 'top' of the field)",
            AutoBuilder.buildAutoChooser());
    autoChooser.addOption("Taxi from middle", new PathPlannerAuto("TaxiMiddle"));
    autoChooser.addOption("Taxi from top", new PathPlannerAuto("TaxiTop"));
    autoChooser.addOption("Taxi from bottom", new PathPlannerAuto("TaxiBottom"));
    autoChooser.addOption("Taxi Flexible", new PathPlannerAuto("TaxiFlexibleAuto"));

    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));

    /*  // Set up SysId routines
        autoChooser.addOption(
            "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
        autoChooser.addOption(
            "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
         autoChooser.addOption(
            "Drive SysId (Quasistatic Forward)",
            drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        autoChooser.addOption(
            "Drive SysId (Quasistatic Reverse)",
            drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        autoChooser.addOption(
            "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
        autoChooser.addOption(
            "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));
        autoChooser.addOption("programmingtestpatplanner", new PathPlannerAuto("programmingtestauto"));
    */
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX()));

    controller.y().whileTrue(Commands.run(() -> drive.synchronizeEncoders(), drive));

    // Lock to 0° when A button is held
    controller
        .a()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -controller.getLeftY(),
                () -> -controller.getLeftX(),
                () -> new Rotation2d(0)));

    // Switch to X pattern when X button is pressed
    controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Reset gyro to 0° when B button is pressed
    controller
        .b()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                    drive)
                .ignoringDisable(true));

    reefPad.button(1).onTrue(arcade.reefAButtonCommand());
    reefPad.button(2).onTrue(arcade.reefBButtonCommand());
    reefPad.button(3).onTrue(arcade.reefCButtonCommand());
    reefPad.button(4).onTrue(arcade.reefDButtonCommand());
    reefPad.button(5).onTrue(arcade.reefEButtonCommand());
    reefPad.button(6).onTrue(arcade.reefFButtonCommand());
    reefPad.button(7).onTrue(arcade.reefGButtonCommand());
    reefPad.button(8).onTrue(arcade.reefHButtonCommand());
    reefPad.button(9).onTrue(arcade.reefIButtonCommand());
    reefPad.button(10).onTrue(arcade.reefJButtonCommand());
    reefPad.button(11).onTrue(arcade.reefKButtonCommand());
    reefPad.button(12).onTrue(arcade.reefLButtonCommand());

    otherPad.button(1).onTrue(arcade.climbButtonCommand());
    otherPad.button(2).onTrue(arcade.leftProcessorButtonCommand());
    otherPad.button(3).onTrue(arcade.rightProcessorButtonCommand());
    otherPad.button(4).onTrue(arcade.leftCoralStationButtonCommand());
    otherPad.button(5).onTrue(arcade.rightCoralStationButtonCommand());
    otherPad.button(6).onTrue(arcade.l1ButtonCommand());
    otherPad.button(7).onTrue(arcade.l2ButtonCommand());
    otherPad.button(8).onTrue(arcade.l3ButtonCommand());
    otherPad.button(9).onTrue(arcade.l4ButtonCommand());
    otherPad.button(10).onTrue(arcade.bargeButtonCommand());

    elevator.setDefaultCommand(
        elevator.manualControlElevatorCommand(() -> armController.getRightY()));

    hand.setDefaultCommand(hand.manualControlHandCommand(() -> armController.getLeftY()));

    hook.setDefaultCommand(hook.manualControlHandCommand(() -> armController.getLeftTriggerAxis()));

    armController.y().whileTrue(hook.hookIntakeCommand());
    armController.x().whileTrue(hook.ejectAlgaeCommand());
    armController.start().whileTrue(hook.kickAlgaeCommand());

    armController.povLeft().whileTrue(elevator.moveL3Command());

    armController.povDown().whileTrue(elevator.moveL2Command());

    armController.povUp().whileTrue(elevator.moveL4Command());

    armController.leftBumper().whileTrue(hand.autoIntakeCommand());

    // armController.leftStick().whileTrue(Commands.run(ScoreAndReplaceCommand, hand, hook,
    // elevator));

    // armController.a().whileTrue(hand.primeEjectCommand());

    armController.rightBumper().whileTrue(hand.ejectCommand());

    armController.b().whileTrue(hand.resetWristCommand());

    climb.setDefaultCommand(
        climb.manualControlClimbCommand(() -> -controller.getRightTriggerAxis()));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
