// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
  /** Creates a new Flywheel. */

  TalonFX motor = new TalonFX(1);
  PIDController PID = new PIDController(45, 0, 4);

  double allowableError = 2;
  double goal = 0;

  public Arm() {

  }

  /**
   * @param position in rotations
   */
  public Command setPosition(double position){
    return this.run(()->{
      motor.setVoltage(PID.calculate(motor.getPosition().getValueAsDouble(),position));
      goal = position;
    });
  }

  public Command stopMotor(){
    return this.run(()->{
      motor.stopMotor();
      goal = 0;
    });
  }

  public double getPosition(){
    return motor.getPosition().getValueAsDouble();
  }

  public double getTarget(){
    return goal;
  }

  public boolean isAtPosition(){
    return Math.abs(getPosition()-goal) < allowableError;
  }

  @Override
  public void periodic() {
    simulationPeriodic();
  }

  private final DCMotorSim m_motorSimModel =
    new DCMotorSim(DCMotor.getFalcon500(1), 32.0, 1);
  
  public void simulationPeriodic() {
    // use the motor voltage to calculate new position and velocity
    // using WPILib's DCMotorSim class for physics simulation
    m_motorSimModel.setInputVoltage(motor.getMotorVoltage().getValueAsDouble());
    m_motorSimModel.update(0.020); // assume 20 ms loop time
    var motorSim = motor.getSimState();
    // apply the new rotor position and velocity to the TalonFX;
    // note that this is rotor position/velocity (before gear ratios)
    motorSim.setRawRotorPosition(m_motorSimModel.getAngularPositionRotations());
    motorSim.setRotorVelocity(
        Units.radiansToRotations(m_motorSimModel.getAngularVelocityRadPerSec())
    );
  }
  
}
