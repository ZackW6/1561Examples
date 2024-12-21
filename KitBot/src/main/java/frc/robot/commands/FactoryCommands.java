// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Robot;
import frc.robot.constants.VisionConstants;
import frc.robot.subsystems.DriveBase;

/** Add your docs here. */
public class FactoryCommands {

    private DriveBase drivetrain;

    private IntSupplier speakerID = ()->{
        if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red){
            return 4;
        }
        return 7;
    };

    public FactoryCommands(DriveBase drivetrain){
        this.drivetrain = drivetrain;
    }

    private PIDController distanceController = new PIDController(.3, 0, .05);

    private PIDController rotationController = new PIDController(.002, 0, .0001);

    // public Command alignToSpeaker(){
    //     return Commands.run(()->{
    //         if (drivetrain.getCamera().getDistToTag(speakerID.getAsInt()).isEmpty() || drivetrain.getCamera().getYawToTag(speakerID.getAsInt()).isEmpty()){
    //             return;
    //         }
    //         if (Math.abs(drivetrain.getCamera().getDistToTag(speakerID.getAsInt()).get()-2) < .05 && Math.abs(drivetrain.getCamera().getYawToTag(speakerID.getAsInt()).get()) < 3) {
    //             return;
    //         }
    //         drivetrain.arcadeDrive(-distanceController.calculate(drivetrain.getCamera().getDistToTag(speakerID.getAsInt()).get(),2)
    //         , -rotationController.calculate(drivetrain.getCamera().getYawToTag(speakerID.getAsInt()).get(),0), false);
    //     });
    // }
}
