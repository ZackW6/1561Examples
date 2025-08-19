package frc.robot.subsystems.intake;

import java.util.function.DoubleSupplier;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.constants.IntakeConstants;
import frc.robot.subsystems.digitalInputs.CANRange;
import frc.robot.subsystems.digitalInputs.DigitalInputIO;
import frc.robot.subsystems.digitalInputs.DigitalInputSim;
import frc.robot.subsystems.digitalInputs.MotorDI;

public class Intake extends SubsystemBase{
    private final FlywheelIO intakeIO;

    private final NetworkTable robot = NetworkTableInstance.getDefault().getTable("Robot");
    private final NetworkTable intakeTable = robot.getSubTable("Intake");

    private final DoublePublisher intakeVelocityPublisher = intakeTable
        .getDoubleTopic("IntakeVelocity").publish();
    private final DoublePublisher intakeTargetPublisher = intakeTable
        .getDoubleTopic("IntakeTargetVelocity").publish();
    private final BooleanPublisher hasCoralPublisher = intakeTable
        .getBooleanTopic("HasCoral").publish();
    private final BooleanPublisher hasAlgaePublisher = intakeTable
        .getBooleanTopic("HasAlgae").publish();

    private final DigitalInputIO coralLaser;
    private final DigitalInputIO algaeSense;

    public Intake(){
        if (Robot.isSimulation()){
            coralLaser = new DigitalInputSim();
            algaeSense = new DigitalInputSim();
            intakeIO = new SimIntake();
        }else{
            coralLaser = new CANRange(IntakeConstants.CORAL_LASER_ID, .09);
            
            intakeIO = new TalonIntake();
            algaeSense = new MotorDI(()->intakeIO.getCurrent(), ()->getTargetVelocity(), 50);
        }
    }

    public void setVelocity(double rps){
        intakeIO.setVelocity(rps);
    }
    
    //set velocity in rotations
    public Command reachGoal(double rps){
        return this.run(() -> intakeIO.setVelocity(rps));
    }

    //set velocity in rotations
    public Command reachGoal (DoubleSupplier rps){
        return this.run(() -> intakeIO.setVelocity(rps.getAsDouble()));
    }

    public Command setVoltage(DoubleSupplier volts){
        return this.run(()->intakeIO.setVoltage(volts.getAsDouble()));
    }
    
    public Command setVoltage(double volts){
        return this.run(()->intakeIO.setVoltage(volts));
    }

    /**
   * Run control loop to reach and maintain goal.
   *
   * @param goal the position to maintain
   */
    public Command reachGoalOnce(double goal) {
        return this.runOnce(()->intakeIO.setVelocity(goal));
    }

    public Command stop(){
        return this.runOnce(()->intakeIO.stop());
    }

    public boolean hasCoral(){
        return coralLaser.getValue();
    }

    public boolean hasAlgae(){
        return algaeSense.getValue();
    }

    public double getVelocity(){
        return intakeIO.getVelocity();
    }

    public double getTargetVelocity(){
        return intakeIO.getTarget();
    }

    public DigitalInputIO getCoralDigitalInputIO(){
        return coralLaser;
    }

    public DigitalInputIO getAlgaeDigitalInputIO(){
        return algaeSense;
    }

    @Override
    public void periodic() {
        intakeVelocityPublisher.accept(getVelocity());
        intakeTargetPublisher.accept(getTargetVelocity());
        hasAlgaePublisher.accept(hasAlgae());
        hasCoralPublisher.accept(hasCoral());
    }
}
