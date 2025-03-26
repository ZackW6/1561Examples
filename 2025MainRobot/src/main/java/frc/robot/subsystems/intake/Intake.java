package frc.robot.subsystems.intake;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANrange;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.networktables.StructSubscriber;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.constants.IntakeConstants;
import frc.robot.subsystems.intake.realIntake.CANRange;
import frc.robot.subsystems.intake.realIntake.DigitalInputLS;
import frc.robot.subsystems.intake.realIntake.MotorDI;
import frc.robot.subsystems.intake.realIntake.TalonIntake;
import frc.robot.subsystems.intake.simIntake.DigitalInputSim;
import frc.robot.subsystems.intake.simIntake.SimIntake;

public class Intake extends SubsystemBase{
    private final FlywheelIO intakeIO;

    //Send data to Network table
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

    //Create limit switches and lazer
    private final DigitalInputIO coralLimitSwitch1;
    private final DigitalInputIO coralLimitSwitch2;
    private final DigitalInputIO coralLaser;
    private final DigitalInputIO algaeSense;

    public Intake(){
        if (Robot.isSimulation()){
            coralLimitSwitch1 = new DigitalInputSim();
            coralLimitSwitch2 = new DigitalInputSim();
            coralLaser = new DigitalInputSim();
            algaeSense = new DigitalInputSim();
            intakeIO = new SimIntake();
        }else{
            coralLimitSwitch1 = new DigitalInputLS(IntakeConstants.CORAL_LIMIT_SWITCH_ID1);
            coralLimitSwitch2 = new DigitalInputLS(IntakeConstants.CORAL_LIMIT_SWITCH_ID2);
            coralLaser = new CANRange(IntakeConstants.CORAL_LASER_ID, .09);
            
            intakeIO = new TalonIntake();
            algaeSense = new MotorDI(()->intakeIO.getCurrent(), ()->intakeIO.getAcceleration(), 50);
        }
    }

    //set velocity in rotations
    public Command setVelocity(DoubleSupplier rps){
        return this.run(() -> intakeIO.setVelocity(rps.getAsDouble()));
    }

    //Update velocity in rotations
    public Command setVelocity(double rps){
        return this.run(() -> intakeIO.setVelocity(rps));
    }

    public Command stop(){
        return this.runOnce(()->intakeIO.stop());
    }

    public boolean hasCoral(){
        return coralLaser.getValue();
        // return coralLimitSwitch1.getValue() || coralLimitSwitch2.getValue() || coralLaser.getValue();
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
    //publish velocity data to network table and is main loop of intake
    public void periodic() {
        intakeVelocityPublisher.accept(getVelocity());
        intakeTargetPublisher.accept(getTargetVelocity());
        hasAlgaePublisher.accept(hasAlgae());
        hasCoralPublisher.accept(hasCoral());
    }
}
