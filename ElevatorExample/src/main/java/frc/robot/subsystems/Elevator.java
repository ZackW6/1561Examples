// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.Slot2Configs;
import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.sim.ChassisReference;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.PWMSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.constants.ElevatorConstants;
import frc.robot.simulationUtil.ElevateSim;

public class Elevator extends SubsystemBase {

  private final DCMotor gearbox = DCMotor.getFalcon500(1);

  private TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration();

  // Standard classes for controlling our elevator
  private final PositionVoltage m_request = new PositionVoltage(0);

  private final TalonFX motorLead = new TalonFX(ElevatorConstants.ELEVATOR_MOTOR1_ID);
  private final TalonFX motorFollow = new TalonFX(ElevatorConstants.ELEVATOR_MOTOR2_ID);
  // private final CANcoder encoder = new CANcoder(ElevatorConstants.ELEVATOR_ENCODER_ID);

  private final double allowableError = .02;

  private final ElevatorSim m_elevatorSim =
      new ElevatorSim(
          gearbox,
          ElevatorConstants.ELEVATOR_GEARING,
          ElevatorConstants.ELEVATOR_MASS_KG,
          ElevatorConstants.ELEVATOR_RADIUS,
          ElevatorConstants.ELEVATOR_MIN_HEIGHT,
          ElevatorConstants.ELEVATOR_MAX_HEIGHT,
          true,
          0);
  private final ElevateSim elevatorSim;
  /** Subsystem constructor. */
  public Elevator() {
    configMotor();

    elevatorSim = new ElevateSim(motorLead, m_elevatorSim);
    elevatorSim.addSimImage("Elevator",.5,ElevatorConstants.ELEVATOR_MAX_HEIGHT);
    // elevatorSim.configureCANCoder(encoder, ChassisReference.CounterClockwise_Positive,-.25);

    ShuffleboardTab shuffleboardTab = Shuffleboard.getTab("Elevator");
      shuffleboardTab.addNumber("Elevator Position Meters",
              () -> getCurrentPosition());
      shuffleboardTab.addBoolean("Is Elevator In Position", this::isAtPosition);
  }

  /**
   * Run control loop to reach and maintain goal.
   *
   * @param goal the position to maintain
   */
  public Command reachGoal(double goal) {
    return this.run(()->motorLead.setControl(m_request.withPosition(goal).withSlot(0)));
  }
  /**
   * Run control loop to reach and maintain changing goal.
   *
   * @param goal the position to maintain
   */
  public Command reachGoal(DoubleSupplier goal) {
    return this.run(()->motorLead.setControl(m_request.withPosition(goal.getAsDouble()).withSlot(0)));
  }

  public double getCurrentPosition(){
    return motorLead.getPosition().getValueAsDouble();
  }

  public boolean isAtPosition(){
    return Math.abs(getCurrentPosition()-motorLead.getClosedLoopReference().getValueAsDouble())<allowableError;
  }

  @Override
  public void periodic(){
    System.out.println(getCurrentPosition());
    if (Robot.isSimulation()){
      elevatorSim.simulationPeriodic();
    }
  }

  private void configMotor(){
    talonFXConfigs = new TalonFXConfiguration();
    // CANcoderConfiguration canCoderConfigs = new CANcoderConfiguration();
    // Set to factory default
    // encoder.getConfigurator().apply(new CANcoderConfiguration());
    motorLead.getConfigurator().apply(new TalonFXConfiguration());
    motorFollow.getConfigurator().apply(new TalonFXConfiguration());
    
    // canCoderConfigs.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
    // canCoderConfigs.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    // canCoderConfigs.MagnetSensor.MagnetOffset = 0;
    
    // encoder.getConfigurator().apply(canCoderConfigs);

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
    // talonFXConfigs.Feedback.FeedbackRemoteSensorID = encoder.getDeviceID();
    // talonFXConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    talonFXConfigs.Feedback.SensorToMechanismRatio = ElevatorConstants.ELEVATOR_GEARING;
    talonFXConfigs.Feedback.RotorToSensorRatio = 1.0;

    // var motionMagicConfigs = talonFXConfigs.MotionMagic;
    // motionMagicConfigs.MotionMagicCruiseVelocity = ElevatorConstants.CRUISE_VELOCITY; // Unlimited cruise velocity
    // motionMagicConfigs.MotionMagicAcceleration = ElevatorConstants.MAX_ACCELERATION;
    // motionMagicConfigs.MotionMagicJerk = ElevatorConstants.JERK;
    talonFXConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
    talonFXConfigs.CurrentLimits.StatorCurrentLimit = 80;
    
    motorLead.getConfigurator().apply(talonFXConfigs);
    motorFollow.getConfigurator().apply(talonFXConfigs);
  }
}
