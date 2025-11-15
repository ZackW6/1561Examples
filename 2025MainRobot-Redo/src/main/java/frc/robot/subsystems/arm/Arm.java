// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.networktables.StructSubscriber;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.subsystems.CommandMechanism;


public class Arm extends SubsystemBase {
  
  private final ArmIO armIO;

  //Send Arm data to NetworkTable
  private final NetworkTable robot = NetworkTableInstance.getDefault().getTable("Robot");
  private final NetworkTable armTable = robot.getSubTable("Arm");

  private final NetworkTable table = robot.getSubTable("Elevator");
  private final StructSubscriber<Pose3d> elevatorSubscriber = table.getStructTopic("ElevatorStage3",Pose3d.struct).subscribe(new Pose3d());

  private final StructPublisher<Pose3d> armPublisher = armTable
    .getStructTopic("ArmAngle", Pose3d.struct).publish();

  private final DoublePublisher armRotations = armTable
    .getDoubleTopic("ArmRotations").publish();
  private final DoublePublisher armTarget = armTable
    .getDoubleTopic("ArmTarget").publish();

  /** Subsystem constructor. */
  public Arm() {
    if (Robot.isSimulation()){
      armIO = new SimArm();
    }else{
      armIO = new TalonArm();
    }
  }

  public void setPosition(double position){
    armIO.setPosition(position);
  }

  /**
   * Run control loop to reach and maintain goal.
   *
   * @param goal the position to maintain
   */
  public Command reachGoal(double goal) {
    return this.run(()->armIO.setPosition(goal));
  }

  /**
   * Run control loop to reach and maintain changing goal.
   *
   * @param goal the position to maintain
   */
  public Command reachGoal(DoubleSupplier goal) {
    return this.run(()->armIO.setPosition(goal.getAsDouble()));
  }

  /**
   * Run control loop to reach and maintain goal.
   *
   * @param goal the position to maintain
   */
  public Command reachGoalOnce(double goal) {
    return this.runOnce(()->armIO.setPosition(goal));
  }

  public Command setVoltage(DoubleSupplier volts){
    return this.run(()->armIO.setVoltage(volts.getAsDouble()));
  }

  public Command setVoltage(double volts){
    return this.run(()->armIO.setVoltage(volts));
  }

  public Command stop(){
    return this.runOnce(()->armIO.stop());
  }

  //Get position of Arm
  public double getPosition(){
    return armIO.getPosition();
  }

  //Get target
  public double getTarget(){
    return armIO.getTarget();
  }

  @Override
  //Send Current position and orientation data of Arm to Network table and get Elevator data, is Main loop of arm
  public void periodic(){
    double height = elevatorSubscriber.get().getZ();
    armPublisher.accept(new Pose3d(CommandMechanism.ELEVATOR_END_EFFECTOR_OFFSETX,0,CommandMechanism.ELEVATOR_END_EFFECTOR_OFFSETZ + height,new Rotation3d(0,Units.rotationsToRadians(getPosition()),0)));
    armRotations.accept(getPosition());
    armTarget.accept(getTarget());
  }
}
