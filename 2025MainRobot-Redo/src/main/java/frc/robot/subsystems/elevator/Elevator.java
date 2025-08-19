// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;


public class Elevator extends SubsystemBase {
  
  private final ElevatorIO elevatorIO;

  private final NetworkTable robot = NetworkTableInstance.getDefault().getTable("Robot");
  private final NetworkTable elevTable = robot.getSubTable("Elevator");

  /**
   * height given by this affects the subscriber in arm, be aware if you want to change the name
   */
  private final StructPublisher<Pose3d> elevatorStage1 = elevTable
    .getStructTopic("ElevatorStage1", Pose3d.struct).publish();
  private final StructPublisher<Pose3d> elevatorStage2 = elevTable
    .getStructTopic("ElevatorStage2", Pose3d.struct).publish();
  private final StructPublisher<Pose3d> elevatorStage3 = elevTable
    .getStructTopic("ElevatorStage3", Pose3d.struct).publish();

  private final DoublePublisher elevatorTotalHeight = elevTable
    .getDoubleTopic("ElevatorHeight").publish();
  private final DoublePublisher elevatorTarget = elevTable
    .getDoubleTopic("ElevatorTarget").publish();

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

  public void setVolts(double volts){
    elevatorIO.setVoltage(volts);
  }

  /**
   * Run control loop to reach and maintain goal.
   *
   * @param goal the position to maintain
   */
  public Command reachGoal(double goal) {
    return this.run(()->elevatorIO.setPosition(goal));
  }
  /**
   * Run control loop to reach and maintain changing goal.
   *
   * @param goal the position to maintain
   */
  public Command reachGoal(DoubleSupplier goal) {
    return this.run(()->elevatorIO.setPosition(goal.getAsDouble()));
  }

  /**
   * Run control loop to reach and maintain goal.
   *
   * @param goal the position to maintain
   */
  public Command reachGoalOnce(double goal) {
    return this.runOnce(()->elevatorIO.setPosition(goal));
  }

  public Command setVoltage(DoubleSupplier volts){
    return this.run(()->elevatorIO.setVoltage(volts.getAsDouble()));
  }

  public Command setVoltage(double volts){
    return this.run(()->elevatorIO.setVoltage(volts));
  }

  public Command stop(){
    return this.runOnce(()->elevatorIO.stop());
  }

  public double getPosition(){
    return elevatorIO.getPosition();
  }

  public double getTarget(){
    return elevatorIO.getTarget();
  }

  @Override
  public void periodic(){
    double position = getPosition();
    elevatorStage1.accept(new Pose3d(0,0,Math.min(position,.7),new Rotation3d()));
    elevatorStage2.accept(new Pose3d(0,0,Math.min(position,1.4),new Rotation3d()));
    elevatorStage3.accept(new Pose3d(0,0,position,new Rotation3d()));
    elevatorTotalHeight.accept(getPosition());
    elevatorTarget.accept(getTarget());
  }
}
