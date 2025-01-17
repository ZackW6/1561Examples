package frc.robot.subsystems.elevator.realElevator;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ArmConstants;
import frc.robot.constants.ElevatorConstants;
import frc.robot.subsystems.elevator.ElevatorIO;

public class TalonElevator implements ElevatorIO{
    private TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration();
    private CurrentLimitsConfigs currentLimits = new CurrentLimitsConfigs();

    private final MotionMagicTorqueCurrentFOC m_request = new MotionMagicTorqueCurrentFOC(0);

    private final TalonFX motorLeader;
    private final TalonFX motorFollower;
    private final CANcoder encoder;

    private final double sensorRatio = ElevatorConstants.ELEVATOR_SENSOR_TO_MECHANISM_RATIO;

    private final double drumCircumferenceMeters = 2 * Math.PI * ElevatorConstants.ELEVATOR_DRUM_RADIUS;

    public TalonElevator(){
        motorLeader = new TalonFX(ElevatorConstants.ELEVATOR_LEADER_ID);
        motorFollower = new TalonFX(ElevatorConstants.ELEVATOR_FOLLOWER_ID);
        encoder = new CANcoder(ElevatorConstants.ELEVATOR_ENCODER_ID);
        motorFollower.setControl(new Follower(motorLeader.getDeviceID(), false));
        configMotor();
    }

    @Override
    public void setPosition(double position) {
        motorLeader.setControl(m_request.withPosition(position).withSlot(0));
    }

    @Override
    public void stop() {
        motorLeader.stopMotor();
    }

    @Override
    public double getPosition() {
        return motorLeader.getPosition().getValueAsDouble();
    }

    @Override
    public double getPositionMeters() {
        return (motorLeader.getPosition().getValueAsDouble())*drumCircumferenceMeters;
    }

    @Override
    public double getTargetPositionMeters() {
        return (motorLeader.getDifferentialClosedLoopReference().getValueAsDouble())*drumCircumferenceMeters;
    }

    private void configMotor(){
        //TODO check this with current CTRE docs, this is a copy of old
        talonFXConfigs = new TalonFXConfiguration();
        CANcoderConfiguration canCoderConfigs = new CANcoderConfiguration();
        // //Set to factory default
        encoder.getConfigurator().apply(new CANcoderConfiguration());
        motorLeader.getConfigurator().apply(new TalonFXConfiguration());
        motorFollower.getConfigurator().apply(new TalonFXConfiguration());
        canCoderConfigs.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1;

        canCoderConfigs.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        canCoderConfigs.MagnetSensor.MagnetOffset = ElevatorConstants.ELEVATOR_ENCODER_OFFSET;
        
        encoder.getConfigurator().apply(canCoderConfigs);

        Slot0Configs slot0Configs = talonFXConfigs.Slot0;
        slot0Configs.kS = ElevatorConstants.S;
        slot0Configs.kV = ElevatorConstants.V;
        slot0Configs.kA = ElevatorConstants.A;
        slot0Configs.kP = ElevatorConstants.P;
        slot0Configs.kI = ElevatorConstants.I;
        slot0Configs.kD = ElevatorConstants.D;
        slot0Configs.kG = ElevatorConstants.G;
        slot0Configs.GravityType = GravityTypeValue.Elevator_Static;

        talonFXConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        talonFXConfigs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;//InvertedValue.Clockwise_Positive
        talonFXConfigs.Feedback.FeedbackRemoteSensorID = encoder.getDeviceID();
        talonFXConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        talonFXConfigs.Feedback.SensorToMechanismRatio = ElevatorConstants.ELEVATOR_SENSOR_TO_MECHANISM_RATIO;
        talonFXConfigs.Feedback.RotorToSensorRatio = ElevatorConstants.ELEVATOR_ROTOR_TO_SENSOR_RATIO;

        var motionMagicConfigs = talonFXConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = ElevatorConstants.CRUISE_VELOCITY; // Unlimited cruise velocity
        motionMagicConfigs.MotionMagicAcceleration = ElevatorConstants.MAX_ACCELERATION;
        motionMagicConfigs.MotionMagicJerk = ElevatorConstants.JERK;

        currentLimits.SupplyCurrentLimit = ElevatorConstants.ELEVATOR_CURRENT_LIMIT; // Limit to 1 amps
        currentLimits.SupplyCurrentLimitEnable = ElevatorConstants.ELEVATOR_ENABLE_CURRENT_LIMIT; // And enable it
        currentLimits.StatorCurrentLimit = ElevatorConstants.ELEVATOR_STATOR_CURRENT_LIMIT; // Limit stator to 20 amps
        currentLimits.StatorCurrentLimitEnable = ElevatorConstants.ELEVATOR_STATOR_CURRENT_LIMIT_ENABLE; // And enable it

        talonFXConfigs.CurrentLimits = currentLimits;

        motorLeader.getConfigurator().apply(talonFXConfigs);
        motorFollower.getConfigurator().apply(talonFXConfigs);
    }
    
}
