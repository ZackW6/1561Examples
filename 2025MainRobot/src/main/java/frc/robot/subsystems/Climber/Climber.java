package frc.robot.subsystems.climber;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
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
import frc.robot.subsystems.climber.realClimber.TalonClimber;
import frc.robot.subsystems.climber.simClimber.SimClimber;

public class Climber extends SubsystemBase {
  
    private final ArmIO climberIO;

    //Send Climber data to NetworkTable
    private final NetworkTable robot = NetworkTableInstance.getDefault().getTable("Robot");
    private final NetworkTable climberTable = robot.getSubTable("Climber");
  
    private final StructPublisher<Pose3d> climberPublisher = climberTable
      .getStructTopic("ClimberAngle", Pose3d.struct).publish();
  
    /** Subsystem constructor. */
    public Climber() {
      if (Robot.isSimulation()){
        climberIO = new SimClimber();
      }else{
        climberIO = new TalonClimber();
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
  
    //Reach Goal Position in Rotations
    public Command reachGoal(double goal) {
      return this.run(()->climberIO.setPosition(goal));
    }
    /**
     * Run control loop to reach and maintain goal.
     *
     * @param goal the position to maintain
     */

    //Reach goal position in Degrees
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
  
    public double getPosition(){
      return climberIO.getPosition();
    }
    
    public double getTarget(){
      return climberIO.getTarget();
    }

    //Send current climber position and orientation data to Network table
    @Override
    public void periodic(){
      climberPublisher.accept(new Pose3d(0,-.31,.14,new Rotation3d(Units.rotationsToRadians(getPosition()) - Math.PI/2,0,0)));
    }
  }
  
