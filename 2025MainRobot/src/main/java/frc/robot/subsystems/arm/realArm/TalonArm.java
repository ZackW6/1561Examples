package frc.robot.subsystems.arm.realArm;

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
import frc.robot.constants.ArmConstants;
import frc.robot.subsystems.arm.ArmIO;

public class TalonArm implements ArmIO{

    //Instance of TalonConfig
    private TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration();
    //Instance of Current Limits
    private CurrentLimitsConfigs currentLimits = new CurrentLimitsConfigs();

    //Magic
    private final MotionMagicTorqueCurrentFOC m_request = new MotionMagicTorqueCurrentFOC(0);
    // private final PositionVoltage m_request = new PositionVoltage(0);

    private final TalonFX armMotor;
    private final CANcoder encoder;

    private final double armPretendOffset = 0;

    //Config Motor to correct IDs
    public TalonArm(){
        armMotor = new TalonFX(ArmConstants.ARM_MOTOR_ID);
        encoder = new CANcoder(ArmConstants.ARM_ENCODER_ID);
        configMotor();
    }
    
    @Override
    public void setPosition(double position) {
        armMotor.setControl(m_request.withPosition(position + armPretendOffset).withSlot(0));
    }

    @Override
    public void stop() {
        armMotor.stopMotor();
    }

    @Override
    public double getPosition() {
        return armMotor.getPosition().getValueAsDouble()+armPretendOffset;
    }

    @Override
    public double getTarget() {
        return armMotor.getDifferentialClosedLoopReference().getValueAsDouble()+armPretendOffset;
    }

    private void configMotor(){
        //TODO check this with current CTRE docs, this is a copy of old
        talonFXConfigs = new TalonFXConfiguration();
        CANcoderConfiguration canCoderConfigs = new CANcoderConfiguration();
        // //Set to factory default
        encoder.getConfigurator().apply(new CANcoderConfiguration());
        armMotor.getConfigurator().apply(new TalonFXConfiguration());
        
        canCoderConfigs.MagnetSensor.AbsoluteSensorDiscontinuityPoint = .5;
        // canCoderConfigs.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
        canCoderConfigs.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        canCoderConfigs.MagnetSensor.MagnetOffset = ArmConstants.ANGLE_OFFSET.getRotations();
        
        encoder.getConfigurator().apply(canCoderConfigs);

        Slot0Configs slot0Configs = talonFXConfigs.Slot0;
        slot0Configs.kS = ArmConstants.kS; // Add 0.25 V output to overcome static friction
        slot0Configs.kV = ArmConstants.kV; // A velocity target of 1 rps results in 0.12 V output
        slot0Configs.kA = ArmConstants.kA; // An acceleration of 1 rps/s requires 0.01 V output
        slot0Configs.kP = ArmConstants.kP; //4080.564;// A position error of 2.5 rotations results in 12 V output
        slot0Configs.kI = ArmConstants.kI; //0; // no output for integrated error
        slot0Configs.kD = ArmConstants.kD; //60.082; // A velocity error of 1 rps results in 0.1 V output
        slot0Configs.kG = ArmConstants.kG;
        slot0Configs.GravityType = GravityTypeValue.Arm_Cosine;

        talonFXConfigs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;//InvertedValue.Clockwise_Positive
        talonFXConfigs.Feedback.FeedbackRemoteSensorID = encoder.getDeviceID();
        talonFXConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        talonFXConfigs.Feedback.SensorToMechanismRatio = ArmConstants.ARM_SENSOR_TO_MECHANISM_RATIO;
        talonFXConfigs.Feedback.RotorToSensorRatio = ArmConstants.ARM_ROTOR_TO_SENSOR_RATIO;
        
        /* Motion Magic Settings */
        var motionMagicConfigs = talonFXConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = ArmConstants.CRUISE_VELOCITY; // Unlimited cruise velocity
        motionMagicConfigs.MotionMagicAcceleration = ArmConstants.MAX_ACCELERATION;
        motionMagicConfigs.MotionMagicJerk = ArmConstants.JERK;
        // motionMagicConfigs.MotionMagicExpo_kV = 0.12; // kV is around 0.12 V/rps
        // motionMagicConfigs.MotionMagicExpo_kA = 0.1; // Use a slower kA of 0.1 V/(rps/s
        /* Current Limits */
        currentLimits.SupplyCurrentLimit = ArmConstants.ARM_CURRENT_LIMIT; // Limit to 1 amps
        // m_currentLimits.SupplyCurrentThreshold = ArmConstants.ARM_SUPPLY_CURRENT_THRESHOLD; // If we exceed 4 amps
        // m_currentLimits.SupplyTimeThreshold = ArmConstants.ARM_CURRENT_THRESHOLD_TIME; // For at least 1 second
        currentLimits.SupplyCurrentLimitEnable = ArmConstants.ARM_ENABLE_CURRENT_LIMIT; // And enable it
        currentLimits.StatorCurrentLimit = ArmConstants.ARM_STATOR_CURRENT_LIMIT; // Limit stator to 20 amps
        currentLimits.StatorCurrentLimitEnable = ArmConstants.ARM_STATOR_CURRENT_LIMIT_ENABLE; // And enable it

        talonFXConfigs.CurrentLimits = currentLimits;

        armMotor.getConfigurator().apply(talonFXConfigs);
    }
}
