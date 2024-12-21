// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.simulation;

import java.util.ArrayList;

import com.ctre.phoenix6.Utils;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Time;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.motorcontrol.PWMMotorController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.constants.DriveBaseConstants;

/** Add your docs here. */
public class SimMotor extends SubsystemBase{

    private double gearing = 1;
    private double jkgMetersSquared = .001;
    private DCMotor gearbox = DCMotor.getNEO(1);
    private FlywheelSim motorSim = new FlywheelSim(gearbox, gearing, jkgMetersSquared);

    private double maxVoltage = 12;
    private double setSpeed = 0;
    private double frictionVelocity = 0;
    private double maxSpeed = Double.MAX_VALUE;

    private double positionRotations = 0;

    private CANSparkMax realMotor;

    private final ArrayList<SimMotor> m_followers = new ArrayList<>();

    private boolean inverted = false;
    private int ID;

    public SimMotor(int ID, MotorType motorType){
        this.ID = ID;
        if (Robot.isSimulation()){

            return;
        }
        realMotor = new CANSparkMax(ID, motorType);
    }

    /**
     * rps
     * @param frictionVelocity
     */
    public void setFrictionVelocity(double frictionVelocity){
        this.frictionVelocity = frictionVelocity;
    }

    /**
     * rps
     * @param maxSpeed
     */
    public void setMaxSpeed(double maxSpeed){
        this.maxSpeed = maxSpeed;
    }

    /**
     * Set the PWM value.
     *
     * <p>The PWM value is set using a range of -1.0 to 1.0, appropriately scaling the value for the
     * FPGA.
     *
     * @param speed The speed value between -1.0 and 1.0 to set.
     */
    public void set(double speed) {
        if (!Robot.isSimulation()){
            realMotor.set(speed * (inverted ? -1.0 : 1.0));
            for (SimMotor follower : m_followers){
                follower.realMotor.set(speed * (inverted ? -1.0 : 1.0));
            }
            return;
        }
        setSpeed = speed * (inverted ? -1.0 : 1.0);
        for (SimMotor follower : m_followers){
            follower.setSpeed = speed * (inverted ? -1.0 : 1.0);
        }
    }

    /**
     * Get the recently set value of the PWM. This value is affected by the inversion property. If you
     * want the value that is sent directly to the MotorController, use {@link
     * edu.wpi.first.wpilibj.PWM#getSpeed()} instead.
     *
     * @return The most recently set value for the PWM between -1.0 and 1.0.
     */
    public double get() {
        if (Robot.isSimulation()){
            return setSpeed;
        }
        return realMotor.get();
    }

    public void setInverted(boolean isInverted) {
        inverted = isInverted;
    }

    public boolean getInverted() {
        return inverted;
    }

    public void disable() {
        if (Robot.isSimulation()){
            motorSim.setInputVoltage(0);
            return;
        }
        realMotor.disable();
    }

    public void stopMotor() {
        if (Robot.isSimulation()){
            this.motorSim.setInputVoltage(0);
            return;
        }
        
        realMotor.stopMotor();
    }

    public void setSimjkgMetersSquared(double amount){
        double radPerSec = motorSim.getAngularVelocityRadPerSec();
        this.motorSim = new FlywheelSim(gearbox, gearing, amount);
        this.motorSim.setState(radPerSec);
    }

    public void setGearing(double amount){
        double radPerSec = motorSim.getAngularVelocityRadPerSec();
        this.motorSim = new FlywheelSim(gearbox, amount, jkgMetersSquared);
        this.motorSim.setState(radPerSec);
    }

    public void setGearbox(DCMotor gearbox){
        double radPerSec = motorSim.getAngularVelocityRadPerSec();
        this.motorSim = new FlywheelSim(gearbox, gearing, jkgMetersSquared);
        this.motorSim.setState(radPerSec);
    }

    /**
     * Make the given PWM motor controller follow the output of this one.
     *
     * @param follower The motor controller follower.
     */
    public void addFollower(SimMotor follower) {
        if (Robot.isSimulation()){
            m_followers.add(follower);
            return;
        }
        follower.realMotor.follow(realMotor);
    }

    private void addFriction() {
        if (Math.abs(motorSim.getAngularVelocityRPM()/60) < frictionVelocity) {
            motorSim.setState(0);
        } else if (motorSim.getAngularVelocityRPM()/60 > 0.0) {
            motorSim.setState(Units.rotationsToRadians(motorSim.getAngularVelocityRPM()/60-frictionVelocity));
        } else {
            motorSim.setState(Units.rotationsToRadians(motorSim.getAngularVelocityRPM()/60+frictionVelocity));
        }
    }

    private void capSpeed() {
        if (motorSim.getAngularVelocityRPM()/60 > maxSpeed) {
            motorSim.setState(Units.rotationsToRadians(maxSpeed));
        } else if (motorSim.getAngularVelocityRPM()/60 < -maxSpeed){
            motorSim.setState(Units.rotationsToRadians(-maxSpeed));
        }
    }

    /**
     * Gets the PWM channel number.
     *
     * @return The channel number.
     */
    public int getChannel() {
        return ID;
    }

    /**
     * in rotations
     * affected by inversion
     * @return
     */
    public double getSimPosition(){
        return positionRotations * (inverted ? -1.0 : 1.0);
    }

    private double lastTime = Utils.getCurrentTimeSeconds();
    @Override
    public void periodic() {
        if (Robot.isSimulation()){
            double speed = setSpeed;
            
            motorSim.setInput(speed*maxVoltage);

            for (var follower : m_followers) {
                follower.set(speed);
            }

            positionRotations += (motorSim.getAngularVelocityRPM()/60)*(Utils.getCurrentTimeSeconds()-lastTime);
            motorSim.update(Utils.getCurrentTimeSeconds()-lastTime);
            lastTime = Utils.getCurrentTimeSeconds();

            addFriction();
            capSpeed();
        }
    }
}
