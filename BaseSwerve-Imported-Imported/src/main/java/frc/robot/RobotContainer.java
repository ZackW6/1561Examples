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

import java.io.IOException;
import java.util.Set;

import org.json.simple.parser.ParseException;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.pathplanner.lib.util.FileVersionException;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.util.ChoreoEX;
import frc.robot.util.DynamicObstacle;
public class RobotContainer {

  private SendableChooser<Command> autoChooser;
  
  private double MaxSpeed = TunerConstants.kSpeedAt12VoltsMps;
  private double MaxAngularRate = TunerConstants.MAX_ANGULAR_RATE;

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final CommandXboxController driverController = new CommandXboxController(0);
  
  private final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(TunerConstants.TRANSLATIONAL_DEADBAND).withRotationalDeadband(TunerConstants.ROTATIONAL_DEADBAND)
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); 

  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

  private final Telemetry logger = new Telemetry(MaxSpeed);

  private void configureBindings() {    
    drivetrain.setDefaultCommand(
        drivetrain.applyRequest(() -> drive.withVelocityX(-driverController.getLeftY() * 1.00 * MaxSpeed)
            .withVelocityY(-driverController.getLeftX() * 1.00 * MaxSpeed)
            .withRotationalRate(-driverController.getRightX()/*driverController.getRawAxis(2)*/ * 2/3 * MaxAngularRate)
    ));

    /* Controller Bindings */

    driverController.y().onTrue(drivetrain.runOnce(() -> drivetrain.setOperatorPerspectiveForward(drivetrain.getYaw())));
    
    driverController.rightTrigger(.5).whileTrue(drivetrain.applyRequest(() -> brake));

    // drivetrain.registerTelemetry(logger::telemeterize);

    //TODO was deffered, i switched to not, make sure it works in all scenarios
    new Trigger(()->DriverStation.isTeleop()).onTrue(Commands.runOnce(()->{
      // DynamicObstacle.setDynamicObstacles("testNodeSize", drivetrain.getPose().getTranslation());
      DynamicObstacle.clearDynamicObstacles(drivetrain.getPose().getTranslation());
      if (Robot.isSimulation()){
        drivetrain.setOperatorPerspectiveForward(Rotation2d.fromDegrees(90));
        return;
      }
      if (DriverStation.getAlliance().get() == Alliance.Red){
        drivetrain.setOperatorPerspectiveForward(Rotation2d.fromDegrees(180));
      }else{
        drivetrain.setOperatorPerspectiveForward(Rotation2d.fromDegrees(0));
      }
    }));
  }

  public RobotContainer() {
    drivetrain.configurePathPlanner();

    configureAutonomousCommands();
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);

    configureBindings();

    //How you might make a choreo only path
    // autoChooser.addOption("ChoreoPath", ChoreoEX.getChoreoGroupPath(true,new String[]{"shootPreAmp","intake4","shoot4M","intake5","shoot5M","intake6","shoot6M","intake7","shoot7M"}));
  }

  public void configureAutonomousCommands() {
    NamedCommands.registerCommand("intake", Commands.none());
    NamedCommands.registerCommand("shoot", Commands.none());
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}