package frc.robot.subsystems.climb.realClimber;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ClimberConstants;
import frc.robot.subsystems.arm.ArmIO;

public class TalonClimber implements ArmIO{
    
    //Instance of TalonConfig
    private TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration();
    //Instance of Current Limits
    private CurrentLimitsConfigs currentLimits = new CurrentLimitsConfigs();

    //Magic
    private final MotionMagicTorqueCurrentFOC m_request = new MotionMagicTorqueCurrentFOC(0);
    // private final PositionVoltage m_request = new PositionVoltage(0);

    private final TalonFX armMotor;

    //Config Motor to correct IDs
    public TalonClimber(){
        armMotor = new TalonFX(ClimberConstants.CLIMBER_MOTOR_ID,"Canivore");
        configMotor();
    }

    @Override
    public void setPosition(double position) {
        armMotor.setControl(m_request.withPosition(position).withSlot(0));
    }

    @Override
    public void stop() {
        armMotor.stopMotor();
    }

    @Override
    public double getPosition() {
        return armMotor.getPosition().getValueAsDouble();
    }

    @Override
    public double getTarget() {
        return (armMotor.getClosedLoopReference().getValueAsDouble());
    }

    private void configMotor(){
        //TODO check this with current CTRE docs, this is a copy of old
        talonFXConfigs = new TalonFXConfiguration();
        // //Set to factory default
        armMotor.getConfigurator().apply(new TalonFXConfiguration());


        Slot0Configs slot0Configs = talonFXConfigs.Slot0;
        slot0Configs.kS = ClimberConstants.kS; // Add 0.25 V output to overcome static friction
        slot0Configs.kV = ClimberConstants.kV; // A velocity target of 1 rps results in 0.12 V output
        slot0Configs.kA = ClimberConstants.kA; // An acceleration of 1 rps/s requires 0.01 V output
        slot0Configs.kP = ClimberConstants.kP; //4080.564;// A position error of 2.5 rotations results in 12 V output
        slot0Configs.kI = ClimberConstants.kI; //0; // no output for integrated error
        slot0Configs.kD = ClimberConstants.kD; //60.082; // A velocity error of 1 rps results in 0.1 V output
        slot0Configs.kG = ClimberConstants.kG;
        slot0Configs.GravityType = GravityTypeValue.Arm_Cosine;

        Slot1Configs slot1Configs = talonFXConfigs.Slot1;
        slot1Configs.kS = ClimberConstants.kS; // Add 0.25 V output to overcome static friction
        slot1Configs.kV = ClimberConstants.kV; // A velocity target of 1 rps results in 0.12 V output
        slot1Configs.kA = ClimberConstants.kA; // An acceleration of 1 rps/s requires 0.01 V output
        slot1Configs.kP = ClimberConstants.kP; // A position error of 2.5 rotations results in 12 V output
        slot1Configs.kI = ClimberConstants.kI; // no output for integrated error
        slot1Configs.kD = ClimberConstants.kD; // A velocity error of 1 rps results in 0.1 V output
        slot1Configs.kG = ClimberConstants.kG;
        slot1Configs.GravityType = GravityTypeValue.Arm_Cosine;
        talonFXConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        talonFXConfigs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;//InvertedValue.Clockwise_Positive
        // talonFXConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        talonFXConfigs.Feedback.SensorToMechanismRatio = ClimberConstants.CLIMBER_SENSOR_TO_MECHANISM_RATIO;
        talonFXConfigs.Feedback.RotorToSensorRatio = ClimberConstants.CLIMBER_ROTOR_TO_SENSOR_RATIO;
        
        /* Motion Magic Settings */
        var motionMagicConfigs = talonFXConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = ClimberConstants.CRUISE_VELOCITY; // Unlimited cruise velocity
        motionMagicConfigs.MotionMagicAcceleration = ClimberConstants.MAX_ACCELERATION;
        motionMagicConfigs.MotionMagicJerk = ClimberConstants.JERK;
        // motionMagicConfigs.MotionMagicExpo_kV = 0.12; // kV is around 0.12 V/rps
        // motionMagicConfigs.MotionMagicExpo_kA = 0.1; // Use a slower kA of 0.1 V/(rps/s
        /* Current Limits */
        currentLimits.SupplyCurrentLimit = ClimberConstants.CLIMBER_CURRENT_LIMIT; // Limit to 1 amps
        // m_currentLimits.SupplyCurrentThreshold = ClimberConstants.CLIMBER_SUPPLY_CURRENT_THRESHOLD; // If we exceed 4 amps
        // m_currentLimits.SupplyTimeThreshold = ClimberConstants.CLIMBER_CURRENT_THRESHOLD_TIME; // For at least 1 second
        currentLimits.SupplyCurrentLimitEnable = ClimberConstants.CLIMBER_ENABLE_CURRENT_LIMIT; // And enable it
        currentLimits.StatorCurrentLimit = ClimberConstants.CLIMBER_STATOR_CURRENT_LIMIT; // Limit stator to 20 amps
        currentLimits.StatorCurrentLimitEnable = ClimberConstants.CLIMBER_STATOR_CURRENT_LIMIT_ENABLE; // And enable it

        talonFXConfigs.CurrentLimits = currentLimits;

        armMotor.getConfigurator().apply(talonFXConfigs);
    }
}
