// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.hardwareSimUtil;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

/** Add your docs here. */
public class ElevateSim{
    private final ElevatorSim elevatorSim;
    private boolean includeCANCoder = false;
    private final TalonFXSimState motorSim;
    private CANcoderSimState encoderSim;
    private double offset;

    public ElevateSim(TalonFX motor, ElevatorSim elevatorSim){
        this.elevatorSim = elevatorSim;
        motorSim = motor.getSimState();
    }

    /**
     * offset will likely be different by .25 incriment rotations compared to correct offset.
     * @param orientation
     * @param offset
     */
    public void configureCANCoder(CANcoder encoder, ChassisReference orientation, double offset){
        includeCANCoder = true;
        encoderSim = encoder.getSimState();
        encoderSim.Orientation = orientation;
        this.offset = offset;
    }

    public void simulationPeriodic() {
        
        // In this method, we update our simulation of what our elevator is doing
        // First, we set our "inputs" (voltages)
        elevatorSim.setInput(motorSim.getMotorVoltage());

        // Next, we update it. The standard loop time is 20ms.
        elevatorSim.update(0.020);

        if (includeCANCoder){
            encoderSim.setRawPosition(elevatorSim.getPositionMeters()-offset);
            encoderSim.setVelocity(elevatorSim.getVelocityMetersPerSecond());
        }
        motorSim.setRawRotorPosition(elevatorSim.getPositionMeters()-offset);
        motorSim.setRotorVelocity(Units.radiansToRotations(Units.radiansToRotations(elevatorSim.getVelocityMetersPerSecond())));
        // SimBattery estimates loaded battery voltages
        // elevatorRoot.setPosition(10, elevatorSim.getPositionMeters()-10);
        // elevatorLigament.setLength(elevatorSim.getPositionMeters());
        // RoboRioSim.setVInVoltage(
        //     BatterySim.calculateDefaultBatteryLoadedVoltage(elevatorSim.getCurrentDrawAmps()));
    }
}
