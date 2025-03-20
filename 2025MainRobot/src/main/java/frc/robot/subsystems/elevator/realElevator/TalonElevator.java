package frc.robot.subsystems.elevator.realElevator;

import static edu.wpi.first.units.Units.Volt;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.Slot2Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ArmConstants;
import frc.robot.constants.ElevatorConstants;
import frc.robot.subsystems.elevator.ElevatorIO;

public class TalonElevator implements ElevatorIO{

    //Instance of Talon Configs
    private TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration();
    //Instance of Limits
    private CurrentLimitsConfigs currentLimits = new CurrentLimitsConfigs();

    //Magic
    private final MotionMagicTorqueCurrentFOC m_request = new MotionMagicTorqueCurrentFOC(0);
    // private final PositionVoltage m_request = new PositionVoltage(0);

    //Initialize motors and encoder
    private final TalonFX motorLeader;
    private final TalonFX motorFollower;

    // private final Notifier PIDAdjuster;
    // private final CANcoder encoder;

    private double targetPosition = 0;

    public TalonElevator(){
        //Create and set motors and encoders up
        motorLeader = new TalonFX(ElevatorConstants.ELEVATOR_LEADER_ID,"Canivore");
        motorFollower = new TalonFX(ElevatorConstants.ELEVATOR_FOLLOWER_ID,"Canivore");
        // encoder = new CANcoder(ElevatorConstants.ELEVATOR_ENCODER_ID);
        motorFollower.setControl(new Follower(motorLeader.getDeviceID(), true));
        configMotor();
        
        // PIDAdjuster = new Notifier(this::periodic);
        // PIDAdjuster.startPeriodic(.01);
        // Runtime.getRuntime().addShutdownHook(new Thread(PIDAdjuster::close));
    }

    @Override
    public void setPosition(double position) {
        setPosition(position,0);
    }

    @Override
    public void applyVoltage(double volts) {
        motorLeader.setControl(new VoltageOut(volts));
    }

    @Override
    public void setPosition(double position, int slot) {
        targetPosition = position;
        motorLeader.setControl(m_request.withPosition(position).withSlot(slot));
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
    public double getTarget() {
        return targetPosition;
        // return (motorLeader.getClosedLoopReference().getValueAsDouble());
    }

    private void configMotor(){
        //TODO check this with current CTRE docs, this is a copy of old
        talonFXConfigs = new TalonFXConfiguration();
        // CANcoderConfiguration canCoderConfigs = new CANcoderConfiguration();
        // //Set to factory default
        // encoder.getConfigurator().apply(new CANcoderConfiguration());
        motorLeader.getConfigurator().apply(new TalonFXConfiguration());
        motorFollower.getConfigurator().apply(new TalonFXConfiguration());
        // canCoderConfigs.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1;

        // canCoderConfigs.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        // canCoderConfigs.MagnetSensor.MagnetOffset = ElevatorConstants.ELEVATOR_ENCODER_OFFSET;
        
        // encoder.getConfigurator().apply(canCoderConfigs);

        Slot0Configs slot0Configs = talonFXConfigs.Slot0;
        slot0Configs.kS = ElevatorConstants.S1;
        slot0Configs.kV = ElevatorConstants.V1;
        slot0Configs.kA = ElevatorConstants.A1;
        slot0Configs.kP = ElevatorConstants.P1;
        slot0Configs.kI = ElevatorConstants.I1;
        slot0Configs.kD = ElevatorConstants.D1;
        slot0Configs.kG = ElevatorConstants.G1;
        slot0Configs.GravityType = GravityTypeValue.Elevator_Static;

        // Slot1Configs slot1Configs = talonFXConfigs.Slot1;
        // slot1Configs.kS = ElevatorConstants.S2;
        // slot1Configs.kV = ElevatorConstants.V2;
        // slot1Configs.kA = ElevatorConstants.A2;
        // slot1Configs.kP = ElevatorConstants.P2;
        // slot1Configs.kI = ElevatorConstants.I2;
        // slot1Configs.kD = ElevatorConstants.D2;
        // slot1Configs.kG = ElevatorConstants.G2;
        // slot1Configs.GravityType = GravityTypeValue.Elevator_Static;

        // Slot2Configs slot2Configs = talonFXConfigs.Slot2;
        // slot2Configs.kS = ElevatorConstants.S3;
        // slot2Configs.kV = ElevatorConstants.V3;
        // slot2Configs.kA = ElevatorConstants.A3;
        // slot2Configs.kP = ElevatorConstants.P3;
        // slot2Configs.kI = ElevatorConstants.I3;
        // slot2Configs.kD = ElevatorConstants.D3;
        // slot2Configs.kG = ElevatorConstants.G3;
        // slot2Configs.GravityType = GravityTypeValue.Elevator_Static;

        talonFXConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        talonFXConfigs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;//InvertedValue.Clockwise_Positive
        // talonFXConfigs.Feedback.FeedbackRemoteSensorID = encoder.getDeviceID();
        // talonFXConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
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

    // private void periodic(){
    //     motorLeader.setControl(m_request.withSlot(0));
    //     // if (getPosition() < getTarget()){
    //     //     if (getPosition() < ElevatorConstants.CARRIAGE_POSITION){
    //     //         motorLeader.setControl(m_request.withSlot(0));
    //     //     }else if (getPosition() < ElevatorConstants.STAGE_ONE_POSITION){
    //     //         motorLeader.setControl(m_request.withSlot(1));
    //     //     }else{
    //     //         motorLeader.setControl(m_request.withSlot(2));
    //     //     }
    //     // }else{
    //     //     if (getPosition() < ElevatorConstants.CARRIAGE_POSITION){
    //     //         motorLeader.setControl(m_request.withSlot(2));
    //     //     }else if (getPosition() < ElevatorConstants.STAGE_ONE_POSITION){
    //     //         motorLeader.setControl(m_request.withSlot(1));
    //     //     }else{
    //     //         motorLeader.setControl(m_request.withSlot(0));
    //     //     }
    //     // }
    // }
}