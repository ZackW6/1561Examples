package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Elevator;

public class RobotContainer {

  private final CommandXboxController driverController = new CommandXboxController(0);

  private final Elevator elevator = new Elevator();

  private void configureBindings() {
    elevator.setDefaultCommand(elevator.reachGoal(()->driverController.getRawAxis(1)*-1+1));
    driverController.a().whileTrue(elevator.reachGoal(1.8));
    driverController.b().whileTrue(elevator.reachGoal(.2));
  }

  public RobotContainer() {
    configureBindings();
  }

  public Command getAutonomousCommand() {
    // TODO no auto, this is just the elevator example
    return Commands.none();
  }
}