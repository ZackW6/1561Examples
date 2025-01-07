// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.simulationUtil;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.constants.IntakeConstants;

/** Add your docs here. */
public class FlyWheelSim {

    private final FlywheelSim flywheelSim;
    private final TalonFXSimState motorSim;

    private double currentRad = 0;
    private double currentTime = 0;

    public FlyWheelSim(TalonFX motor, FlywheelSim flywheelSim){
        this.flywheelSim = flywheelSim;
        this.motorSim = motor.getSimState();
    }
    
    public void simulationPeriodic() {
        // In this method, we update our simulation of what our arm is doing
        // First, we set our "inputs" (voltages)
        flywheelSim.setInput(motorSim.getMotorVoltage());
        // // Next, we update it. The standard loop time is 20ms.
        flywheelSim.update(0.02);
        //Makes the rest of the robot react based on this usage of Voltage
        motorSim.setRawRotorPosition(Units.radiansToRotations(currentRad));
        motorSim.setRotorVelocity(Units.radiansToRotations(flywheelSim.getAngularVelocityRadPerSec()));
        // Update the Mechanism Arm angle based on the simulated arm angle
        currentRad+=(flywheelSim.getAngularVelocityRadPerSec()*(Timer.getFPGATimestamp()-currentTime));
        currentTime = Timer.getFPGATimestamp();
        // RoboRioSim.setVInVoltage(
        //     BatterySim.calculateDefaultBatteryLoadedVoltage(flywheelSim.getCurrentDrawAmps()));
        
    }

    public double getCurrentPosition(){
        return currentRad;
    }
}
