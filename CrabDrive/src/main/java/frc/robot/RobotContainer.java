 // final Optional<EstimatedRobotPose> optionalEstimatedPoseRight = vision1.update();
        // if (optionalEstimatedPoseRight.isPresent()) {
        //     final EstimatedRobotPose estimatedPose = optionalEstimatedPoseRight.get();          
        //     poseEstimator.updateVisionMeasurement(estimatedPose.toPose2d(), estimatedPose.timestampSeconds);
        // }
        
        // final Optional<EstimatedRobotPose> optionalEstimatedPoseLeft = vision2.update();
        // if (optionalEstimatedPoseLeft.isPresent()) {
        //     final EstimatedRobotPose estimatedPose = optionalEstimatedPoseLeft.get();          
        //     poseEstimator.updateVisionMeasurement(estimatedPose.toPose2d(), estimatedPose.timestampSeconds);
        // }
        
        // poseEstimator.update(/*ccw gyro rotation*/, /*module positions array*/);// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.FactoryCommands;
import frc.robot.motors.types.talon.TalonDrive;
import frc.robot.motors.types.talon.TalonSteer;
import frc.robot.subsystems.DriveBase;
import frc.robot.util.ChoreoEX;
public class RobotContainer {

  private SendableChooser<Command> autoChooser = new SendableChooser<>();
  
  private final CommandXboxController driverController = new CommandXboxController(0);

  private final DriveBase driveBase = new DriveBase(new TalonSteer(0, 0)
      ,List.of(new TalonDrive(1, 1),new TalonDrive(2, 2)
      ,new TalonDrive(3, 3),new TalonDrive(4, 4)));

  private final Telemetry logger = new Telemetry();

  private final FactoryCommands factoryCommands = new FactoryCommands();

  private void configureBindings() {

    driveBase.registerTelemetry(logger::telemeterize);
  }

  public RobotContainer() {

    SmartDashboard.putData("Auto Chooser", autoChooser);
    
    configureBindings();
    configureAutonomousCommands();
  }

  public void configureAutonomousCommands() {
    autoChooser.addOption("nothing", Commands.none());
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}