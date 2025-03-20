package frc.robot.subsystems.ramp;

import java.util.function.DoubleSupplier;

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
import frc.robot.subsystems.arm.ArmIO;
import frc.robot.subsystems.arm.realArm.TalonArm;
import frc.robot.subsystems.arm.simArm.SimArm;
import frc.robot.subsystems.climb.realClimber.TalonClimber;
import frc.robot.subsystems.climb.simClimber.SimClimber;
import frc.robot.subsystems.ramp.realRamp.TalonRamp;
import frc.robot.subsystems.ramp.simRamp.SimRamp;

public class Ramp extends SubsystemBase {
  
    private final ArmIO climberIO;
  
    //Send climber data to network table
    private final NetworkTable robot = NetworkTableInstance.getDefault().getTable("Robot");
    private final NetworkTable rampTable = robot.getSubTable("Ramp");
  
    private final StructPublisher<Pose3d> rampPublisher = rampTable
      .getStructTopic("RampAngle", Pose3d.struct).publish();
    private final DoublePublisher rampAngle = rampTable
      .getDoubleTopic("TrueRampAngle").publish();
    private final DoublePublisher rampTarget = rampTable
      .getDoubleTopic("TrueRampTarget").publish();
    /** Subsystem constructor. */
    public Ramp() {
      if (Robot.isSimulation()){
        climberIO = new SimRamp();
      }else{
        climberIO = new TalonRamp();
      }
    }
    
    public void setPosition(double position){
        climberIO.setPosition(position);
    }
  
    /**
     * Run control loop to reach and maintain goal.
     *
     * @param goal the position to maintain
     */

    //Reach goal position in rotations
    public Command reachGoal(double goal) {
      return this.run(()->climberIO.setPosition(goal));
    }
    /**
     * Run control loop to reach and maintain goal.
     *
     * @param goal the position to maintain
     */

     //Reach goal in Degrees
    public Command reachGoalDegrees(double goal) {
      return this.run(()->climberIO.setPosition(Units.degreesToRotations(goal)));
    }
    /**
     * Run control loop to reach and maintain changing goal.
     *
     * @param goal the position to maintain
     */

    //Update goal value
    public Command reachGoal(DoubleSupplier goal) {
      return this.run(()->climberIO.setPosition(goal.getAsDouble()));
    }

    public Command stop(){
      return this.run(()->climberIO.stop());
    }
  
    public double getPosition(){
      return climberIO.getPosition();
    }
    
    public double getTarget(){
      return climberIO.getTarget();
    }
  
    //Send current position and orientation data of Ramp to network table and updates position, is main loop of ramp
    @Override
    public void periodic(){
      rampPublisher.accept(new Pose3d(-.25,0,0.8,new Rotation3d(0,Units.rotationsToRadians(getPosition()+.189),0)));
      rampAngle.accept(getPosition());
      rampTarget.accept(getTarget());
    }
  }
  