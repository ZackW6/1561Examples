// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

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
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.networktables.StructSubscriber;
import edu.wpi.first.networktables.Subscriber;
import edu.wpi.first.util.struct.Struct;
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
import frc.robot.constants.ArmConstants;
import frc.robot.subsystems.arm.realArm.TalonArm;
import frc.robot.subsystems.arm.simArm.SimArm;
import frc.robot.subsystems.elevator.realElevator.TalonElevator;
import frc.robot.subsystems.elevator.simElevator.SimElevator;


public class Arm extends SubsystemBase {
  
  private final ArmIO armIO;

  //Send Arm data to NetworkTable
  private final NetworkTable robot = NetworkTableInstance.getDefault().getTable("Robot");
  private final NetworkTable armTable = robot.getSubTable("Arm");

  private final NetworkTable table = robot.getSubTable("Elevator");
  private final StructSubscriber<Pose3d> elevatorSubscriber = table.getStructTopic("ElevatorStage3",Pose3d.struct).subscribe(new Pose3d());

  private final StructPublisher<Pose3d> armPublisher = armTable
    .getStructTopic("ArmAngle", Pose3d.struct).publish();

  private final DoublePublisher trueArmAngle = armTable
    .getDoubleTopic("TrueArmAngle").publish();
  private final DoublePublisher armTarget = armTable
    .getDoubleTopic("TrueArmTarget").publish();

  /** Subsystem constructor. */
  public Arm() {
    if (Robot.isSimulation()){
      armIO = new SimArm();
    }else{
      armIO = new TalonArm();
    }
  }

  //Set position of arm
  public void setPosition(double position){
    armIO.setPosition(position);
  }
  
  /**
   * Run control loop to reach and maintain goal.
   *
   * @param goal the position to maintain
   */

  //Set Arm goal in Rotations
  public Command reachGoal(double goal) {
    return this.run(()->armIO.setPosition(goal));
  }
  /**
   * Run control loop to reach and maintain goal.
   *
   * @param goal the position to maintain
   */

  //Set arm goal in Degrees
  public Command reachGoalDegrees(double goal) {
    return this.run(()->armIO.setPosition(Units.degreesToRotations(goal)));
  }
  /**
   * Run control loop to reach and maintain goal.
   *
   * @param goal the position to maintain
   */
  public Command reachGoalDegrees(DoubleSupplier goal) {
    return this.run(()->armIO.setPosition(Units.degreesToRotations(goal.getAsDouble())));
  }
  /**
   * Run control loop to reach and maintain goal.
   *
   * @param goal the position to maintain
   */
  public Command reachGoalDegreesOnce(double goal) {
    return this.runOnce(()->armIO.setPosition(Units.degreesToRotations(goal)));
  }
  /**
   * Run control loop to reach and maintain goal.
   *
   * @param goal the position to maintain
   */
  public Command reachGoalOnce(double goal) {
    return this.runOnce(()->armIO.setPosition(goal));
  }
  /**
   * Run control loop to reach and maintain changing goal.
   *
   * @param goal the position to maintain
   */

  //Update goal values
  public Command reachGoal(DoubleSupplier goal) {
    return this.run(()->armIO.setPosition(goal.getAsDouble()));
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
    armPublisher.accept(new Pose3d(.262,0,.487 + height,new Rotation3d(0,Units.rotationsToRadians(getPosition()+.441),0)));
    trueArmAngle.accept(getPosition());
    armTarget.accept(getTarget());
  }
}
