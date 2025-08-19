package frc.robot.subsystems.climb;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.subsystems.arm.ArmIO;

public class Climber extends SubsystemBase {
  
    private final ArmIO climberIO;

    //Send Climber data to NetworkTable
    private final NetworkTable robot = NetworkTableInstance.getDefault().getTable("Robot");
    private final NetworkTable climberTable = robot.getSubTable("Climber");
  
    private final StructPublisher<Pose3d> climberPublisher = climberTable
      .getStructTopic("ClimberAngle", Pose3d.struct).publish();

    private final DoublePublisher climberRotations = climberTable
      .getDoubleTopic("ClimberRotations").publish();

    private final DoublePublisher climberTarget = climberTable
      .getDoubleTopic("ClimberTarget").publish();
  
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
     * Run control loop to reach and maintain changing goal.
     *
     * @param goal the position to maintain
     */

    //Update goal value
    public Command reachGoal(DoubleSupplier goal) {
      return this.run(()->climberIO.setPosition(goal.getAsDouble()));
    }

    /**
     * Run control loop to reach and maintain goal.
     *
     * @param goal the position to maintain
    */
    public Command reachGoalOnce(double goal) {
      return this.runOnce(()->climberIO.setPosition(goal));
    }
    
    public Command setVoltage(DoubleSupplier volts){
      return this.run(()->climberIO.setVoltage(volts.getAsDouble()));
    }
  
    public Command setVoltage(double volts){
      return this.run(()->climberIO.setVoltage(volts));
    }

    public Command stop(){
      return this.runOnce(()->climberIO.stop());
    }
  
    public double getPosition(){
      return climberIO.getPosition();
    }
    
    public double getTarget(){
      return climberIO.getTarget();
    }

    //Send current climber position and orientation data to Network table, is main loop of Climber
    @Override
    public void periodic(){
      climberPublisher.accept(new Pose3d(0,-.31,.14,new Rotation3d(Units.rotationsToRadians(getPosition()) - Math.PI/2,0,0)));
      climberRotations.accept(getPosition());
      climberTarget.accept(getTarget());
    }
  }
  
