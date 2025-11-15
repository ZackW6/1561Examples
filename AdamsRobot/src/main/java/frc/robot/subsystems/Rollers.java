// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Rollers extends SubsystemBase {
  /** Creates a new Rollers. */
  private final TalonFX motor;
  public Rollers() {
    motor = new TalonFX(11, "Canivore");
    setDefaultCommand(move2(0));
  }

  public void move(double a) {
    motor.setControl(new VoltageOut(a));
  }

  public Command move2(double b){
    return this.run(()->move(b)).alongWith(Commands.print(b+""));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
  }
}
