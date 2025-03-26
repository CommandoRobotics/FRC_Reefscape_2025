// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.hand.Hand;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ScoreCoralL4Command extends Command {

  Hand hand;
  boolean isFinished = false;
  double accumulatedPower;
  double accumulatedTime;

  /** Creates a new WaitForCoralCommand. */
  public ScoreCoralL4Command(Hand hand) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.hand = hand;
    accumulatedPower = 0;
    accumulatedTime = 0;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    accumulatedPower = 0;
    accumulatedTime = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    accumulatedPower += .05;
    accumulatedTime += .02;

    if (accumulatedTime <= 1) {

      isFinished = false;
      hand.manualHand(-1 * accumulatedPower);

    } else if (accumulatedTime > 1 && accumulatedTime <= 2) {
      hand.Eject();
      isFinished = false;
    } else {
      isFinished = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    accumulatedPower = 0;
    accumulatedTime = 0;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
