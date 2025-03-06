// Commando Robotics - FRC 5889
// Arcade - Logic for the custom game controller

package frc.robot.subsystems.arcade;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

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

  @Override
  public void periodic() {
    // Post values to dashboard for debugging.
    SmartDashboard.putString("Arcade Mode", currentMode.toString());
    SmartDashboard.putString("Arcade Target Location", currentTargetLocation.toString());
    SmartDashboard.putString("Arcade Target Height", currentTargetHeight.toString());
  }

  @Override
  public void simulationPeriodic() {}
}
