package frc.robot.subsystems.intake.realIntake;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.constants.ArmConstants;
import frc.robot.constants.IntakeConstants;
import frc.robot.subsystems.intake.FlywheelIO;

public class TalonIntake implements FlywheelIO{
    
    //Get instance of talon configs
    private TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration();
    //Get instance of Limits
    private CurrentLimitsConfigs currentLimits = new CurrentLimitsConfigs();

    //Magic
    // private final MotionMagicVelocityTorqueCurrentFOC torqueCurrentFOC = new MotionMagicVelocityTorqueCurrentFOC(0);
    private final VelocityVoltage torqueCurrentFOC = new VelocityVoltage(0);

    private final TalonFX intakeMotor;

    public TalonIntake(){
        intakeMotor = new TalonFX(IntakeConstants.INTAKE_MOTOR_ID);
        configMotor();
    }

    @Override
    public void setVelocity(double rps) {
        intakeMotor.setControl(torqueCurrentFOC.withVelocity(rps).withSlot(0));
    }

    @Override
    public void stop() {
        intakeMotor.stopMotor();
    }

    @Override
    public double getVelocity() {
        return intakeMotor.getVelocity().getValueAsDouble();
    }

    @Override
    public double getTarget() {
        return (intakeMotor.getDifferentialClosedLoopReference().getValueAsDouble());
    }

    @Override
    public double getAcceleration() {
        return intakeMotor.getAcceleration().getValueAsDouble();
    }

    @Override
    public double getCurrent() {
        return intakeMotor.getStatorCurrent().getValueAsDouble();
    }
    
    private void configMotor(){
        talonFXConfigs = new TalonFXConfiguration();
        Slot0Configs slot0Configs = talonFXConfigs.Slot0;
        slot0Configs.kS = IntakeConstants.kS;
        slot0Configs.kV = IntakeConstants.kV;
        slot0Configs.kA = IntakeConstants.kA;
        slot0Configs.kP = IntakeConstants.kP;
        slot0Configs.kI = IntakeConstants.kI;
        slot0Configs.kD = IntakeConstants.kD;

        talonFXConfigs.Feedback.SensorToMechanismRatio = IntakeConstants.INTAKE_SENSOR_TO_MECHANISM_RATIO;

        talonFXConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        // set Motion Magic Velocity settings
        MotionMagicConfigs motionMagicConfigs = talonFXConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicAcceleration = 10000; // Target acceleration of 400 rps/s (0.25 seconds to max)
        motionMagicConfigs.MotionMagicJerk = 0; // Target jerk of 4000 rps/s/s (0.1 seconds)
    
        currentLimits = new CurrentLimitsConfigs();
        currentLimits.SupplyCurrentLimit = IntakeConstants.INTAKE_CURRENT_LIMIT; // Limit to 1 amps
        currentLimits.SupplyCurrentLimitEnable = IntakeConstants.INTAKE_ENABLE_CURRENT_LIMIT; // And enable it

        currentLimits.StatorCurrentLimit = IntakeConstants.INTAKE_STATOR_CURRENT_LIMIT; // Limit stator to 20 amps
        currentLimits.StatorCurrentLimitEnable = IntakeConstants.INTAKE_STATOR_CURRENT_LIMIT_ENABLE; // And enable it

        talonFXConfigs.CurrentLimits = currentLimits;

        intakeMotor.getConfigurator().apply(talonFXConfigs);
    }
}
