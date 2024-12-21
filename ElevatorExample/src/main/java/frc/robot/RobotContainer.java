package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.constants.ElevatorConstants;
import frc.robot.subsystems.Elevator;

public class RobotContainer {

  private final CommandXboxController driverController = new CommandXboxController(0);

  private final Elevator elevator = new Elevator();

  private void configureBindings() {
    elevator.setDefaultCommand(elevator.reachGoal(()->(Math.abs(driverController.getRawAxis(1))*ElevatorConstants.MAX_HEIGHT)/2));
    driverController.a().whileTrue(elevator.reachGoal(ElevatorConstants.MAX_HEIGHT));
    driverController.b().whileTrue(elevator.reachGoal(ElevatorConstants.MAX_HEIGHT/2));
  }

  public RobotContainer() {
    configureBindings();
  }

  public Command getAutonomousCommand() {
    // TODO no auto, this is just the elevator example
    return Commands.none();
  }
}