package frc.robot.subsystems.intake;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
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
import frc.robot.subsystems.intake.realIntake.DigitalInputLS;
import frc.robot.subsystems.intake.realIntake.TalonIntake;
import frc.robot.subsystems.intake.simIntake.DigitalInputSim;
import frc.robot.subsystems.intake.simIntake.SimIntake;

public class Intake extends SubsystemBase{
    private final FlywheelIO intakeIO;

    //Make sure Intake has a piece  
    private final Notifier hasPieceChecker;

    //Send data to Network table
    private final NetworkTable robot = NetworkTableInstance.getDefault().getTable("Robot");
    private final NetworkTable intakeTable = robot.getSubTable("Intake");

    private final DoublePublisher intakeVelocityPublisher = intakeTable
        .getDoubleTopic("IntakeVelocity").publish();
    private final DoublePublisher intakeTargetPublisher = intakeTable
        .getDoubleTopic("IntakeTargetVelocity").publish();

    //Create limit switches and laser
    private final DigitalInputIO coralLimitSwitch1;
    private final DigitalInputIO coralLimitSwitch2;
    private final DigitalInputIO coralLaser;

    //initially set has piece to false
    private boolean hasPiece = false;
    private double lastTime = Timer.getFPGATimestamp();

    public Intake(){
        if (Robot.isSimulation()){
            coralLimitSwitch1 = new DigitalInputSim();
            coralLimitSwitch2 = new DigitalInputSim();
            coralLaser = new DigitalInputSim();
            intakeIO = new SimIntake();
        }else{
            coralLimitSwitch1 = new DigitalInputLS(IntakeConstants.CORAL_LIMIT_SWITCH_ID1);
            coralLimitSwitch2 = new DigitalInputLS(IntakeConstants.CORAL_LIMIT_SWITCH_ID2);
            coralLaser = new DigitalInputLS(IntakeConstants.CORAL_LASER_ID);
            intakeIO = new TalonIntake();
        }
        hasPieceChecker = new Notifier(()->{
            boolean temp = false;
            //if has coral, set temp true and get time otherwise do nothing
            if (coralLimitSwitch2.getValue() || coralLimitSwitch1.getValue()){
                temp = true;
                lastTime = Timer.getFPGATimestamp();
            }else{
                if (Timer.getFPGATimestamp() - lastTime > .4 && Math.abs(intakeIO.getVelocity()) > 5){
                    temp = false;
                }
            }
            hasPiece = coralLaser.getValue() || temp;
        });
        //Check if piece every 2 ms
        hasPieceChecker.startPeriodic(0.02);

        Runtime.getRuntime().addShutdownHook(new Thread(hasPieceChecker::close));
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
        return hasPiece;
        // return coralLimitSwitch1.getValue() || coralLimitSwitch2.getValue() || coralLaser.getValue();
    }

    //Make sure there is coral
    public boolean definiteCoral(){
        return coralLaser.getValue();
    }

    public boolean hasAlgae(){
        return intakeIO.getCurrent() > 10 && Math.abs(intakeIO.getAcceleration()) < .1 && Math.abs(intakeIO.getVelocity() - intakeIO.getTarget()) > .2;
    }

    public double getVelocity(){
        return intakeIO.getVelocity();
    }

    public double getTargetVelocity(){
        return intakeIO.getTarget();
    }

    public DigitalInputIO getCoralDigitalInputIO(){
        return coralLimitSwitch1;
    }

    // public DigitalInputIO getAlgaeDigitalInputIO(){
    //     return algaeLimitSwitch;
    // }

    @Override
    //publish velocity data to network table and is main loop of intake
    public void periodic() {
        intakeVelocityPublisher.accept(getVelocity());
        intakeTargetPublisher.accept(getTargetVelocity());
    }
}
