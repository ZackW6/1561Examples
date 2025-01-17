// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.Slot2Configs;
import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.sim.ChassisReference;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
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
import frc.robot.subsystems.elevator.realElevator.TalonElevator;
import frc.robot.subsystems.elevator.simElevator.SimElevator;


public class Elevator extends SubsystemBase {
  
  private final ElevatorIO elevatorIO;

  private final NetworkTable robot = NetworkTableInstance.getDefault().getTable("Robot");
  private final NetworkTable elevTable = robot.getSubTable("Elevator");

  /**
   * height given by this affects the subscriber in arm, be aware if you want to change the name
   */
  private final StructPublisher<Pose3d> elevatorPublisher = elevTable
    .getStructTopic("ElevatorHeight", Pose3d.struct).publish();

  /** Subsystem constructor. */
  public Elevator() {
    if (Robot.isSimulation()){
      elevatorIO = new SimElevator();
    }else{
      elevatorIO = new TalonElevator();
    }
  }

  public void setPosition(double position){
    elevatorIO.setPosition(position);
  }

  /**
   * Run control loop to reach and maintain goal.
   *
   * @param goal the position to maintain
   */
  public Command reachGoal(double goal) {
    return Commands.runOnce(()->elevatorIO.setPosition(goal));
  }
  /**
   * Run control loop to reach and maintain changing goal.
   *
   * @param goal the position to maintain
   */
  public Command reachGoal(DoubleSupplier goal) {
    return Commands.run(()->elevatorIO.setPosition(goal.getAsDouble()));
  }

  public double getPositionMeters(){
    return elevatorIO.getPositionMeters();
  }

  public double getPosition(){
    return elevatorIO.getPosition();
  }

  @Override
  public void periodic(){
    elevatorPublisher.accept(new Pose3d(0,0,getPositionMeters(),new Rotation3d()));
  }
}
