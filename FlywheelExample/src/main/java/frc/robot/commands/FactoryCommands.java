// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import frc.robot.subsystems.Flywheel;

/** Add your docs here. */
public class FactoryCommands {
    private final Flywheel flywheel;
    public FactoryCommands(Flywheel flywheel){
        this.flywheel = flywheel;
    }
    public Command getFlywheelCommand(){
        return flywheel.stopMotors();
    }
}
