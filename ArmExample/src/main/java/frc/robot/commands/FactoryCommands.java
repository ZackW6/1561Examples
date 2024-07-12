// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Arm.ArmState;

/** Add your docs here. */
public class FactoryCommands {
    private final Arm arm;
    public FactoryCommands(Arm arm){
        this.arm = arm;
    }
    public Command getArmMoveCommand(){
        return Commands.race(arm.setArmRotation(ArmState.Amp),Commands.waitSeconds(.3))
            .andThen(Commands.deadline(Commands.waitSeconds(.5),arm.setArmRotation(ArmState.AmpMove)))
            .finallyDo(()->arm.setDefaultCommand(arm.setArmRotation(ArmState.Speaker)))
            .withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
    }
}
