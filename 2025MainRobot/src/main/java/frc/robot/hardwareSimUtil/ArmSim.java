// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.hardwareSimUtil;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.hardwareSimUtil.mechanismUtil.ArmMech;

/** Add your docs here. */
public class ArmSim{
    private boolean includeCANCoder = false;
    private CANcoderSimState encoderSim;
    private TalonFXSimState motorSim;
    private double offset;

    private final SingleJointedArmSim armSim;

    public ArmSim(TalonFX motor, SingleJointedArmSim armSim){
        this.armSim = armSim;
        motorSim = motor.getSimState();
    }

    /**
     * offset requires encoder offset, because the encoder offset will be active in sim, and we need to cancel it
     * secondly 0 is straight down, so if you have 0 somewhere else you input the opposite of the needed rotation to get there
     * an example would be if your 0 is at a graphs 0, you would need to add .25, as adding is counterclockwise
     * (Im not sure if this works for different orientations..., I hope so)
     * @param encoder
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
        // In this method, we update our simulation of what our arm is doing
        // First, we set our "inputs" (voltages)

        armSim.setInput(motorSim.getMotorVoltage());

        armSim.update(0.02);

        if (includeCANCoder){
            encoderSim.setRawPosition(Units.radiansToRotations(armSim.getAngleRads())-offset);
            encoderSim.setVelocity(Units.radiansToRotations(armSim.getVelocityRadPerSec()));
        }
        motorSim.setRawRotorPosition(Units.radiansToRotations(armSim.getAngleRads())-offset);
        motorSim.setRotorVelocity(Units.radiansToRotations(Units.radiansToRotations(armSim.getVelocityRadPerSec())));
        // encoder.setPosition(Units.radiansToRotations(m_armSim.getAngleRads()));
        // System.out.println(encoder.getPosition().getValue());
        // // Finally, we set our simulated encoder's readings and simulated battery voltage
        // // SimBattery estimates loaded battery voltages

        //Makes the rest of the robot react based on this usage of Voltage
        // RoboRioSim.setVInVoltage(
        //     BatterySim.calculateDefaultBatteryLoadedVoltage(armSim.getCurrentDrawAmps()));

        // Update the Mechanism Arm angle based on the simulated arm angle
        // arm.setAngle(Units.radiansToDegrees(armSim.getAngleRads()));
    }
}
