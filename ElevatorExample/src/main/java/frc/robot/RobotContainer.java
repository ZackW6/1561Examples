package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Elevator;

public class RobotContainer {

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final CommandXboxController driverController = new CommandXboxController(0);

  /* Subsystems */
  private final Elevator elevator = new Elevator();

  private void configureBindings() {
    elevator.setDefaultCommand(elevator.reachGoal(()->driverController.getRawAxis(1)*-25+25));
    driverController.a().whileTrue(elevator.reachGoal(45));
    driverController.b().whileTrue(elevator.reachGoal(5));
  }

  public RobotContainer() {
    configureBindings();
  }

  public Command getAutonomousCommand() {
    // TODO Auto-generated method stub
    return Commands.none();
  }
}