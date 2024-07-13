package frc.robot;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;
import com.choreo.lib.ChoreoTrajectoryState;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.FactoryCommands;
import frc.robot.subsystems.Flywheel;
public class RobotContainer {

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final CommandXboxController driverController = new CommandXboxController(0);

  /* Subsystems */
  private final Flywheel flywheel = new Flywheel();

  private final FactoryCommands factoryCommands = new FactoryCommands(flywheel);

  private void configureBindings() {
    flywheel.setIdleSpeed(0,0);
    driverController.leftBumper().onTrue(flywheel.shootVelocity(60,80).alongWith(Commands.waitSeconds(2)));
    driverController.rightBumper().whileTrue(Commands.run(()->flywheel.shootVelocity(30,30).schedule()));
    driverController.a().onTrue(factoryCommands.getFlywheelCommand());
  }

  public RobotContainer() {
    configureBindings();
  }

  public Command getAutonomousCommand() {
    // TODO Auto-generated method stub
    return Commands.none();
  }
}