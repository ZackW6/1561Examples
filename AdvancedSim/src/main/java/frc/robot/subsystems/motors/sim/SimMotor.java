package frc.robot.subsystems.motors.sim;

import java.util.Optional;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.LinearSystemSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.simulation.VirtualSubsystem;
import frc.robot.subsystems.motors.BaseMotor;

public class SimMotor extends VirtualSubsystem implements BaseMotor{

    private double time = Timer.getFPGATimestamp();

    boolean velocityOrPosition = false;
    boolean isInverted = false;
    double position = 0;
    double goalVelocity = 0;
    double goalPosition = 0;

    private double maxVoltage = 12;

    private double gearing = 1;
    private double jkgMetersSquared = .001;
    private double frictionCoef = .1;
    private DCMotor gearbox = DCMotor.getKrakenX60(1);
    private FlywheelSim motorSim = new FlywheelSim(gearbox, gearing, jkgMetersSquared);

    private int id;

    public SimMotor(int id, double gearing, double jkgMetersSquared, double frictionCoef, DCMotor gearbox){
        this.id = id;
        this.gearing = gearing;
        this.jkgMetersSquared = jkgMetersSquared;
        this.frictionCoef = frictionCoef;
        this.gearbox = gearbox;
        this.motorSim = new FlywheelSim(gearbox, gearing, jkgMetersSquared);
    }

    /**
     * only one thing at a time
     */
    @Override
    public void set(double x) {
        double inputVoltage = Math.max(Math.min(x,1),-1)*maxVoltage;
        if (inputVoltage < 0){
            inputVoltage += frictionCoef;
        }else{
            inputVoltage -= frictionCoef;
        }
        if (Math.abs(inputVoltage) < frictionCoef){
            inputVoltage = 0;
        }
        motorSim.setInputVoltage((inputVoltage * (isInverted ? -1 : 1)));
    }

    @Override
    public double getVelocity() {
        return Units.radiansToRotations(motorSim.getAngularVelocityRadPerSec()) * (isInverted ? -1 : 1);
    }

    @Override
    public double getPosition() {
        return position * (isInverted ? -1 : 1);
    }

    public double getFixedPosition(){
        double fixPos = getPosition()%1;
        if (fixPos > .5){
            fixPos-=1;
        }else if(fixPos < -.5){
            fixPos+=1;
        }
        return fixPos;
    }

    /**
     * both points in the form from -.5 to .5
     * @param pos
     * @param goal
     * @return
     */
    public double distToCorrectedPoint(double pos, double goal){
        double current = ((pos + 1) % 2.0) - 1;
        double target = ((goal + 1) % 2.0) - 1;

        double direct = target - current;
        double wrapped = (target - current + 2.0) % 2.0 - 1;

        if (Math.abs(direct) < Math.abs(wrapped)){
            return direct;
        }
        return wrapped;
    }

    @Override
    public int getMotorID() {
        return id;
    }

    @Override
    public int getEncoderID() {
        return id;
    }

    @Override
    public BaseMotor setInvert(boolean x) {
        isInverted = x;
        return this;
    }

    @Override
    public void update() {
        double deltaTime = Timer.getFPGATimestamp() - time;
        motorSim.update(deltaTime);
        position = position + deltaTime * getVelocity();
        
        time = Timer.getFPGATimestamp();
    }
}
