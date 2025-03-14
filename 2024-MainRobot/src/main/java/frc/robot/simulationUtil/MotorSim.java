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
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
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
public class MotorSim {

    private final TalonFXSimState motorSim;
    private final DCMotorSim motorSimModel;

    private String name;

    public MotorSim(TalonFX motor, DCMotor dcMotor){
        this.motorSim = motor.getSimState();
        motorSimModel =
        new DCMotorSim(dcMotor, 1.0, 0.001);

    }

    private Mechanism2d mech2d;
    private MechanismRoot2d pivot;

    private MechanismRoot2d directionPivot;
    private MechanismLigament2d directionLine;
    
    private MechanismLigament2d[] spokes;


    public void addSimImage(String name, int numOfSpokes){
        this.name = name;
        mech2d = new Mechanism2d(60, 60);
        pivot = mech2d.getRoot(name+" Pivot", 30, 30);
        spokes = new MechanismLigament2d[numOfSpokes];

        for (int i = 0; i < spokes.length;i++){
            spokes[i] = pivot.append(
            new MechanismLigament2d(
                name+" Spoke "+i,
                10,
                Units.radiansToDegrees(motorSimModel.getAngularPositionRad())+(360/spokes.length)*i,
                6,
                new Color8Bit(Color.kRed)));
        }
        SmartDashboard.putData(name, mech2d);
    }


    private double lastVelocity = 0;
    public void simulationPeriodic() {
        // In this method, we update our simulation of what our arm is doing
        // First, we set our "inputs" (voltages)
        motorSimModel.setInput(motorSim.getMotorVoltage());
        // // Next, we update it. The standard loop time is 20ms.
        motorSimModel.update(0.02);
        //Makes the rest of the robot react based on this usage of Voltage
        lastVelocity = motorSimModel.getAngularVelocityRadPerSec();
        motorSim.setRawRotorPosition(motorSimModel.getAngularPositionRotations());
        motorSim.setRotorVelocity(Units.radiansToRotations(motorSimModel.getAngularVelocityRadPerSec()));
        

        // Update the Mechanism Arm angle based on the simulated arm angle
        if (spokes[0] != null){
            for (int i = 0; i<spokes.length;i++){
                spokes[i].setAngle(Units.radiansToDegrees(motorSimModel.getAngularPositionRad())+(360/spokes.length)*i);
            }
        }
        try {
            if (motorSimModel.getAngularVelocityRadPerSec()>0){
                directionLine.setColor(new Color8Bit(Math.min((int)motorSimModel.getAngularVelocityRadPerSec()/3,255),0,0));
            }else{
                directionLine.setColor(new Color8Bit(0,0,Math.min(Math.abs((int)motorSimModel.getAngularVelocityRadPerSec()/3),255)));
            }
        } catch (Exception e) {
            return;
        }
        // RoboRioSim.setVInVoltage(
        //     BatterySim.calculateDefaultBatteryLoadedVoltage(flywheelSim.getCurrentDrawAmps()));
        
    }
    public void addDirectionColor(){
        directionPivot = mech2d.getRoot(name+" Direction Pivot", 0, 60);
        directionLine = directionPivot.append(
            new MechanismLigament2d(
                name+" Direction Line",
                60,
                Units.radiansToDegrees(0),
                10,
                new Color8Bit(Color.kRed)));  
    }
}
