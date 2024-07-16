// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FlywheelConstants;

public class Flywheel extends SubsystemBase {
  /** Creates a new Flywheel. */

  TalonFX motor = new TalonFX(FlywheelConstants.motorID);

  MotionMagicVelocityTorqueCurrentFOC request = new MotionMagicVelocityTorqueCurrentFOC(0);

  double allowableError = FlywheelConstants.allowableError;

  double goal = 0;
  public Flywheel() {
    applyConfigs();
  }

  /**
   * in rps
   * @param rps
   */
  public Command setRotationSpeed(double rps){
    goal = rps;
    return this.run(()->{
      motor.setControl(request.withVelocity(rps));
      goal = rps;
    });
  }

  public Command stopMotor(){
    return this.run(()->{
      motor.stopMotor();
      goal = 0;
    });
  }

  public double getVelocity(){
    return motor.getVelocity().getValueAsDouble();
  }

  public boolean isAtSpeed(){
    return Math.abs(getVelocity()-goal) < allowableError;
  }

  @Override
  public void periodic() {
    simulationPeriodic();
  }

  private final DCMotorSim m_motorSimModel =
    new DCMotorSim(DCMotor.getKrakenX60Foc(1), 1.0, 0.05);

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

  public void applyConfigs(){
    // in init function
    var talonFXConfigs = new TalonFXConfiguration();

    // set slot 0 gains
    var slot0Configs = talonFXConfigs.Slot0;
    slot0Configs.kS = 0; // Add 0.25 V output to overcome static friction
    slot0Configs.kV = 0; // A velocity target of 1 rps results in 0.12 V output
    slot0Configs.kA = 0; // An acceleration of 1 rps/s requires 0.01 V output
    slot0Configs.kP = FlywheelConstants.P; // An error of 1 rps results in 0.11 V output
    slot0Configs.kI = FlywheelConstants.I; // no output for integrated error
    slot0Configs.kD = FlywheelConstants.D; // no output for error derivative

    // set Motion Magic Velocity settings
    var motionMagicConfigs = talonFXConfigs.MotionMagic;
    motionMagicConfigs.MotionMagicAcceleration = 400; // Target acceleration of 400 rps/s (0.25 seconds to max)
    motionMagicConfigs.MotionMagicJerk = 4000; // Target jerk of 4000 rps/s/s (0.1 seconds)

    motor.getConfigurator().apply(talonFXConfigs);
  }
}
