package frc.robot.subsystems;

import java.util.Map;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.MotionMagicExpoDutyCycle;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableListener;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Robot;
import frc.robot.constants.FlywheelConstants;
import frc.robot.simulationUtil.FlyWheelSim;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;

import static edu.wpi.first.units.Units.Volts;

public class Flywheel extends SubsystemBase{
    private final TalonFX leftShooterMotor;
    private final TalonFX  rightShooterMotor;

    private double targetFlywheelSpeedL;
    private double targetFlywheelSpeedR;
    private double IdleSpeed = 0;

    private final MotionMagicVelocityTorqueCurrentFOC velocityRequestL = new MotionMagicVelocityTorqueCurrentFOC(0).withSlot(0);
    private final MotionMagicVelocityTorqueCurrentFOC velocityRequestR = new MotionMagicVelocityTorqueCurrentFOC(0).withSlot(0);

    private final FlywheelSim fSimL = new FlywheelSim(DCMotor.getFalcon500(1), 1, .0062);
    private final FlyWheelSim flywheelSimL;
    private final FlywheelSim fSimR = new FlywheelSim(DCMotor.getFalcon500(1), 1, .0062);
    private final FlyWheelSim flywheelSimR;

    public Flywheel(){
        
        leftShooterMotor = new TalonFX(FlywheelConstants.LEFT_SHOOTER_MOTOR_ID);
        rightShooterMotor = new TalonFX(FlywheelConstants.RIGHT_SHOOTER_MOTOR_ID);

        flywheelSimL = new FlyWheelSim(leftShooterMotor, fSimL);
        flywheelSimL.addSimImage("Left Shooter Sim",1,1,new double[]{.2,.4},.1,4);
        flywheelSimL.addDirectionColor();

        flywheelSimR = new FlyWheelSim(rightShooterMotor, fSimR);
        flywheelSimR.addSimImage("Right Shooter Sim",1,1,new double[]{.2,.4},.1,4);
        flywheelSimR.addDirectionColor();

        configMotors();

        ShuffleboardTab shuffleboardTab = Shuffleboard.getTab("Shooter");
        shuffleboardTab.addNumber("Left Flywheel Speed",
                () -> getLeftFlywheelVelocity());
        shuffleboardTab.addNumber("Right Flywheel Speed",
                () -> getRightFlywheelVelocity());
        shuffleboardTab.addNumber("Left Target Flywheel Speed",
                () -> getTargetFlywheelSpeedL());
        shuffleboardTab.addNumber("Right Target Flywheel Speed",
                () -> getTargetFlywheelSpeedR());
        shuffleboardTab.addBoolean("Is Left Flywheel at Speed", this::isLeftFlywheelAtTargetSpeed);
        shuffleboardTab.addBoolean("Is Right Flywheel at Speed", this::isLeftFlywheelAtTargetSpeed);
        shuffleboardTab.addNumber("Idle Speed",
                () -> IdleSpeed);
    }

    @Override
    public void periodic() {
        if (Robot.isSimulation()){
            flywheelSimL.simulationPeriodic();
            flywheelSimR.simulationPeriodic();
        }
    }
    /**
     * 0 stops the motors with a coast
     * @param Rvelocity
     * @param Lvelocity
     */
    public void setIdleSpeed(double Rvelocity, double Lvelocity){
        if (Rvelocity == 0 && Lvelocity == 0){
            setDefaultCommand(stopMotors());
            IdleSpeed = 0;
        }else{
            setDefaultCommand(shootVelocity(Rvelocity, Lvelocity));
            IdleSpeed = (Rvelocity+Lvelocity)/2;
        }
    }

    public Command shootVelocity(double Rvelocity, double Lvelocity){
        this.targetFlywheelSpeedL = Lvelocity;
        this.targetFlywheelSpeedR = Rvelocity;
        return this.runOnce(() -> {
            leftShooterMotor.setControl(velocityRequestL.withVelocity(Lvelocity));
            rightShooterMotor.setControl(velocityRequestR.withVelocity(Rvelocity));
            this.targetFlywheelSpeedL = Lvelocity;
            this.targetFlywheelSpeedR = Rvelocity;
        });
    }

    public Command stopMotors(){
        return this.runOnce(() -> {
            leftShooterMotor.stopMotor();
            rightShooterMotor.stopMotor();
            this.targetFlywheelSpeedL = 0;
            this.targetFlywheelSpeedR = 0;
        });
    }

    public double getTargetFlywheelSpeedL() {
        return targetFlywheelSpeedL;
    }

    public double getTargetFlywheelSpeedR() {
        return targetFlywheelSpeedR;
    }

    public double getLeftFlywheelVelocity(){
        return leftShooterMotor.getVelocity().getValueAsDouble();
    }

    public double getRightFlywheelVelocity(){
        return rightShooterMotor.getVelocity().getValueAsDouble();
    }

    public boolean isLeftFlywheelAtTargetSpeed() {
        return Math.abs(getLeftFlywheelVelocity() - targetFlywheelSpeedL) < FlywheelConstants.FLYWHEEL_ALLOWABLE_ERROR;
    }

    public boolean isRightFlywheelAtTargetSpeed() {
        return Math.abs(getRightFlywheelVelocity() - targetFlywheelSpeedR) < FlywheelConstants.FLYWHEEL_ALLOWABLE_ERROR;
    }

    public boolean isShooterAtTargetSpeed(){
        return isRightFlywheelAtTargetSpeed() && isLeftFlywheelAtTargetSpeed();
    }

    private void configMotors(){
        TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration();
        Slot0Configs slot0Configs = talonFXConfigs.Slot0;
        slot0Configs.kS = FlywheelConstants.kS;
        slot0Configs.kV = FlywheelConstants.kV;
        slot0Configs.kA = FlywheelConstants.kA;
        slot0Configs.kP = FlywheelConstants.kP;
        slot0Configs.kI = FlywheelConstants.kI;
        slot0Configs.kD = FlywheelConstants.kD;

        talonFXConfigs.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        // set Motion Magic Velocity settings
        MotionMagicConfigs motionMagicConfigs = talonFXConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicAcceleration = 400; // Target acceleration of 400 rps/s (0.25 seconds to max)
        motionMagicConfigs.MotionMagicJerk = 4000; // Target jerk of 4000 rps/s/s (0.1 seconds)
    
        CurrentLimitsConfigs currentLimits = new CurrentLimitsConfigs();
        currentLimits.SupplyCurrentLimit = FlywheelConstants.SHOOTER_CURRENT_LIMIT; // Limit to 1 amps
        currentLimits.SupplyCurrentThreshold = FlywheelConstants.SHOOTER_CURRENT_THRESHOLD; // If we exceed 4 amps
        currentLimits.SupplyTimeThreshold = FlywheelConstants.SHOOTER_CURRENT_THRESHOLD_TIME; // For at least 1 second
        currentLimits.SupplyCurrentLimitEnable = FlywheelConstants.SHOOTER_ENABLE_CURRENT_LIMIT; // And enable it

        currentLimits.StatorCurrentLimit = FlywheelConstants.SHOOTER_STATOR_CURRENT_LIMIT; // Limit stator to 20 amps
        currentLimits.StatorCurrentLimitEnable = FlywheelConstants.SHOOTER_STATOR_CURRENT_LIMIT_ENABLE; // And enable it

        talonFXConfigs.CurrentLimits = currentLimits;
        
        rightShooterMotor.getConfigurator().apply(talonFXConfigs);
        leftShooterMotor.getConfigurator().apply(talonFXConfigs);

        leftShooterMotor.setInverted(true);
    }
}