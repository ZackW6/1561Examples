// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Set;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import frc.robot.constants.LimelightConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.util.PoseEX;

/** Add your docs here. */
public class FactoryCommands {
    private final CommandSwerveDrivetrain drivetrain;
    private final CommandXboxController xboxController;
    public FactoryCommands(CommandSwerveDrivetrain arm, CommandXboxController xboxController){
        this.drivetrain = arm;
        this.xboxController = xboxController;
    }
    
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(0).withRotationalDeadband(0)
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    
    private final PIDController distanceControllerSpeaker = new PIDController(1.7, 0, 0.2);
    private final PIDController thetaControllerSpeaker = new PIDController(2,0,0.01);

    public DeferredCommand getInRange() {
        Pose2d speakerPose = LimelightConstants.K_TAG_LAYOUT.getTagPose(7).get().toPose2d();
        if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get().equals(Alliance.Red)){
        speakerPose = LimelightConstants.K_TAG_LAYOUT.getTagPose(4).get().toPose2d();
        }
    
        Pose2d alignPose = speakerPose;

        DoubleSupplier distanceSpeed = ()-> -distanceControllerSpeaker.calculate(drivetrain.getDistanceFromPoseMeters(alignPose), 2.395);

        DoubleSupplier shareableNum = ()->(drivetrain.getYawOffsetDegrees().getDegrees()-drivetrain.getPoseAngle(alignPose).getDegrees()-90)*Math.PI/180;
        DoubleSupplier xAxis = () -> 
        (-Math.sin(shareableNum.getAsDouble()))*distanceSpeed.getAsDouble()
        /Math.max(Math.max(Math.abs(xboxController.getLeftX()* TunerConstants.kSpeedAt12VoltsMps), Math.abs(xboxController.getLeftY()* TunerConstants.kSpeedAt12VoltsMps)),1)
        -xboxController.getLeftY() * TunerConstants.kSpeedAt12VoltsMps;
        DoubleSupplier yAxis = () ->
        (-Math.cos(shareableNum.getAsDouble()))*distanceSpeed.getAsDouble()
        /Math.max(Math.max(Math.abs(xboxController.getLeftX()* TunerConstants.kSpeedAt12VoltsMps), Math.abs(xboxController.getLeftY()* TunerConstants.kSpeedAt12VoltsMps)),1)
        -xboxController.getLeftX() * TunerConstants.kSpeedAt12VoltsMps;
        thetaControllerSpeaker.reset();


        DoubleSupplier rotationalVelocity = () -> {
        if(drivetrain.getAngleFromPose(alignPose).getDegrees()>0){
            return -thetaControllerSpeaker.calculate(drivetrain.getAngleFromPose(alignPose).getDegrees()-180,0);//-7;
        }else{
            return -thetaControllerSpeaker.calculate(drivetrain.getAngleFromPose(alignPose).getDegrees()+180,0);//+7;
        }
        };

        return new DeferredCommand(()->drivetrain.applyRequest(() -> drive.withVelocityX(xAxis.getAsDouble()) 
            .withVelocityY(yAxis.getAsDouble())
            .withRotationalRate(Units.degreesToRadians(rotationalVelocity.getAsDouble()))),Set.of(drivetrain));
    }
}
