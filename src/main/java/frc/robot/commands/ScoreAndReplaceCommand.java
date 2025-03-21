// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.hand.Hand;
import frc.robot.subsystems.hook.Hook;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoreAndReplaceCommand extends SequentialCommandGroup {
  /** Creates a new ScoreAndReplaceCommand. */
  public ScoreAndReplaceCommand(
      Drive driveSubsystem, Hand handSubsystem, Hook hookSubsystem, Elevator elevatorSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new InstantCommand(() -> elevatorSubsystem.moveToDesiredPostion(6), elevatorSubsystem)
            .repeatedly()
            .withTimeout(4),
        new InstantCommand(() -> handSubsystem.manualHand(1), handSubsystem)
            .repeatedly()
            .withTimeout(1.5),
        new ParallelCommandGroup(
                new InstantCommand(() -> handSubsystem.Eject(), handSubsystem),
                new InstantCommand(() -> hookSubsystem.removeAlgaeKick()))
            .repeatedly()
            .withTimeout(2),
        new ParallelCommandGroup(
                new InstantCommand(() -> handSubsystem.stop(), handSubsystem),
                new InstantCommand(() -> hookSubsystem.stop(), hookSubsystem),
                new InstantCommand(() -> elevatorSubsystem.stop(), elevatorSubsystem))
            .repeatedly()
            .withTimeout(10));
  }
}
