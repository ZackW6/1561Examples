// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Flywheel;

/** Add your docs here. */
public class FactoryCommands {
    private final Arm arm;
    private final Flywheel flywheel;

    public FactoryCommands(Arm arm, Flywheel flywheel){
        this.arm = arm;
        this.flywheel = flywheel;
    }

    public Command exampleCommand(){
        return Commands.deadline(Commands.waitSeconds(2), flywheel.setRotationSpeed(-10), arm.setPosition(.5))
            .andThen(Commands.deadline(Commands.waitSeconds(2),arm.setPosition(.4),flywheel.setRotationSpeed(50)));
    }
}
