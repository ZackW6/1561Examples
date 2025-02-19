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

    private final NetworkTable robot = NetworkTableInstance.getDefault().getTable("Robot");
    private final NetworkTable intakeTable = robot.getSubTable("Intake");

    private final DoublePublisher intakeVelocityPublisher = intakeTable
        .getDoubleTopic("IntakeVelocity").publish();
    private final DoublePublisher intakeTargetPublisher = intakeTable
        .getDoubleTopic("IntakeTargetVelocity").publish();

    private final DigitalInputIO coralLimitSwitch;
    private final DigitalInputIO algaeLimitSwitch;

    public Intake(){
        if (Robot.isSimulation()){
            coralLimitSwitch = new DigitalInputSim();
            algaeLimitSwitch = new DigitalInputSim();
            intakeIO = new SimIntake((DigitalInputSim)coralLimitSwitch);
        }else{
            coralLimitSwitch = new DigitalInputLS(IntakeConstants.CORAL_LIMIT_SWITCH_ID);
            algaeLimitSwitch = new DigitalInputLS(IntakeConstants.ALGAE_LIMIT_SWITCH_ID);
            intakeIO = new TalonIntake();
        }
    }

    public Command setVelocity(DoubleSupplier rps){
        return this.run(() -> intakeIO.setVelocity(rps.getAsDouble()));
    }

    public Command setVelocity(double rps){
        return this.run(() -> intakeIO.setVelocity(rps));
    }

    public Command stop(){
        return this.runOnce(()->intakeIO.stop());
    }

    public boolean hasCoral(){
        return coralLimitSwitch.getValue();
    }

    public boolean hasAlgae(){
        return algaeLimitSwitch.getValue();
    }

    public double getVelocity(){
        return intakeIO.getVelocity();
    }

    public double getTargetVelocity(){
        return intakeIO.getTarget();
    }

    public DigitalInputIO getCoralDigitalInputIO(){
        return coralLimitSwitch;
    }

    public DigitalInputIO getAlgaeDigitalInputIO(){
        return algaeLimitSwitch;
    }

    @Override
    public void periodic() {
        intakeVelocityPublisher.accept(getVelocity());
        intakeTargetPublisher.accept(getTargetVelocity());
    }
}
