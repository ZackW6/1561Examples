package frc.robot.subsystems.ramp;

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

public class Ramp extends SubsystemBase {
  
    private final ArmIO rampIO;
  
    //Send climber data to network table
    private final NetworkTable robot = NetworkTableInstance.getDefault().getTable("Robot");
    private final NetworkTable rampTable = robot.getSubTable("Ramp");
  
    private final StructPublisher<Pose3d> rampPublisher = rampTable
      .getStructTopic("RampAngle", Pose3d.struct).publish();
    private final DoublePublisher rampRotations = rampTable
      .getDoubleTopic("RampRotations").publish();
    private final DoublePublisher rampTarget = rampTable
      .getDoubleTopic("RampTarget").publish();
    /** Subsystem constructor. */
    public Ramp() {
      if (Robot.isSimulation()){
        rampIO = new SimRamp();
      }else{
        rampIO = new TalonRamp();
      }
    }
    
    public void setPosition(double position){
        rampIO.setPosition(position);
    }
  
    /**
     * Run control loop to reach and maintain goal.
     *
     * @param goal the position to maintain
     */

    //Reach goal position in rotations
    public Command reachGoal(double goal) {
      return this.run(()->rampIO.setPosition(goal));
    }

    /**
     * Run control loop to reach and maintain changing goal.
     *
     * @param goal the position to maintain
     */

    //Update goal value
    public Command reachGoal(DoubleSupplier goal) {
      return this.run(()->rampIO.setPosition(goal.getAsDouble()));
    }

    /**
     * Run control loop to reach and maintain goal.
     *
     * @param goal the position to maintain
     */
    public Command reachGoalOnce(double goal) {
      return this.runOnce(()->rampIO.setPosition(goal));
    }

    public Command setVoltage(DoubleSupplier volts){
      return this.run(()->rampIO.setVoltage(volts.getAsDouble()));
    }
  
    public Command setVoltage(double volts){
      return this.run(()->rampIO.setVoltage(volts));
    }

    public Command stop(){
      return this.run(()->rampIO.stop());
    }
  
    public double getPosition(){
      return rampIO.getPosition();
    }
    
    public double getTarget(){
      return rampIO.getTarget();
    }
  
    //Send current position and orientation data of Ramp to network table and updates position, is main loop of ramp
    @Override
    public void periodic(){
      rampPublisher.accept(new Pose3d(-.25,0,0.8,new Rotation3d(0,Units.rotationsToRadians(getPosition()+.189),0)));
      rampRotations.accept(getPosition());
      rampTarget.accept(getTarget());
    }
  }
  