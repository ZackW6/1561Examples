// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.FactoryCommands;
import frc.robot.constants.DriveBaseConstants;
import frc.robot.gameConnection.GameConnection;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.motors.sim.SimMotor;
import frc.robot.subsystems.motors.talon.*;
import frc.robot.subsystems.swerve.SwerveBase;
import frc.robot.util.ChoreoEX;

import frc.robot.subsystems.swerve.SwerveModule;
public class RobotContainer {

  private SendableChooser<Command> autoChooser = new SendableChooser<>();
  
  private final CommandXboxController driverController = new CommandXboxController(0);

  private final Telemetry logger = new Telemetry();

  private final FactoryCommands factoryCommands = new FactoryCommands();

  private final SwerveBase drive = DriveBaseConstants.drive;

  private final double maxSpeed = DriveBaseConstants.speedAt12Volts;


  private void configureBindings() {
    if (Robot.isSimulation()){
      GameConnection.initConnection();
    }
    drive.registerTelemetry(logger::telemeterize);
    drive.setDefaultCommand(drive.fieldRelativeDrive(()->-driverController.getLeftY()*maxSpeed
      ,()->-driverController.getLeftX()*maxSpeed,()->-driverController.getRightX()*DriveBaseConstants.maxRotationRad));
    driverController.y().onTrue(Commands.runOnce(()->drive.seedFieldRelative()));
  }

  public RobotContainer() {
    drive.configurePathPlanner();

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
    
    configureBindings();
    configureAutonomousCommands();
  }

  public void configureAutonomousCommands() {
    
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}