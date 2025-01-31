package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import org.ironmaple.simulation.IntakeSimulation.IntakeSide;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import frc.robot.Robot;
import frc.robot.constants.ElevatorConstants;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.Intake;
import frc.robot.util.MapleSimWorld;

/**
 * every call pertaining to main scoring subsystems should be called through here
 */
public class MainMechanism extends SubsystemBase{

    private final double MAX_ARM_ERROR = .05;
    private final double MAX_ELEVATOR_ERROR = .05;
    private final double MAX_SIGNAL_TIME = .03;
    private final double SHOOT_TIME = .3;
    public static final double ELEVATOR_END_DEFFECTOR_OFFSET = .6;

    private final Arm arm;

    private final Intake intake;

    private final Elevator elevator;

    public enum Positions{
        Intake(0,0),
        L1(0,.1),
        L2(35,.5),
        L3(35,1),
        L4(80,1.8),
        AlgaeU(45,.6),
        AlgaeL(45,.3),
        AlgaeN(-20,2),
        AlgaeP(0,0);

        private double armPosition;
        private double elevatorMeters;
        Positions(double armDegrees, double elevatorMeters){
            this.armPosition = armDegrees;
            this.elevatorMeters = elevatorMeters;
        }

        public double armDegrees(){
            return armPosition;
        }

        public double elevatorMeters(){
            return elevatorMeters;
        }
    }

    public enum IntakeSpeeds{
        Idle(0),
        IntakeCoral(7.5),
        IntakeAlgae(-30),
        Shoot(30);

        private double velocity;
        IntakeSpeeds(double velocityRPS){
            this.velocity = velocityRPS;
        }

        public double value(){
            return velocity;
        }
    }

    public MainMechanism(Arm arm, Intake intake, Elevator elevator){
        this.arm = arm;
        this.intake = intake;
        this.elevator = elevator;
        arm.setDefaultCommand(arm.reachGoalDegrees(Positions.Intake.armDegrees()));
        intake.setDefaultCommand(intake.setVelocity(Positions.Intake.elevatorMeters()));
        elevator.setDefaultCommand(elevator.reachGoal(IntakeSpeeds.Idle.value()));
        if (Robot.isSimulation()){
            MapleSimWorld.addIntakeSimulation("CoralIntake", "Coral", IntakeSide.BACK, .5,.4, new Translation2d(-.1,0));
            MapleSimWorld.addShooterSimulation(()->
                new Transform3d(0.2 + (arm.getPosition() > 1.0/7.0 ? .29 : 0),0, elevator.getPositionMeters() + ELEVATOR_END_DEFFECTOR_OFFSET,new Rotation3d(0, (arm.getPosition() > 1.0/7.0 ? -Math.PI/2 : -Units.rotationsToRadians(arm.getPosition())),0))
                , ()->2
                , "Coral"
                , "CoralIntake");
            MapleSimWorld.addIntakeRequirements("CoralIntake", ()->intake.getVelocity() > 5);
            MapleSimWorld.addIntakeRequirements("CoralIntake", ()->Math.abs(arm.getPosition() - Positions.Intake.armPosition) < .1);
            MapleSimWorld.addIntakeRequirements("CoralIntake", ()->Math.abs(elevator.getPositionMeters() - Positions.Intake.elevatorMeters) < .1);
            MapleSimWorld.hasPiece("CoralIntake",(has)->intake.getCoralDigitalInputIO().setValue(has));
            MapleSimWorld.addShootRequirements("CoralIntake", ()->intake.getVelocity() > 10);

            MapleSimWorld.addIntakeSimulation("AlgaeIntake","Algae",IntakeSide.FRONT, .5,.4,new Translation2d(.3,0));
            MapleSimWorld.addIntakeRequirements("AlgaeIntake", ()->intake.getVelocity() < -20);
            MapleSimWorld.hasPiece("AlgaeIntake",(has)->intake.getAlgaeDigitalInputIO().setValue(has));

            MapleSimWorld.addShooterSimulation(()->
                new Transform3d(0.2,0, elevator.getPositionMeters() + ELEVATOR_END_DEFFECTOR_OFFSET,new Rotation3d(0, -Units.rotationsToRadians(arm.getPosition()),0))
                , ()->2
                , "Algae"
                , "AlgaeIntake");
            MapleSimWorld.addShootRequirements("AlgaeIntake", ()->intake.getVelocity() > 10);


        }
    }

    public Command idle(){
        return arm.reachGoalDegrees(Positions.Intake.armDegrees())
            .alongWith(elevator.reachGoal(Positions.Intake.elevatorMeters()))
            .alongWith(intake.setVelocity(IntakeSpeeds.Idle.value()));
    }

    /**
     * ends after you get a piece
     * @return
     */
    public Command intake(){
        return Commands.parallel(arm.reachGoalDegrees(Positions.Intake.armDegrees())
            ,elevator.reachGoal(Positions.Intake.elevatorMeters())
            ,intake.setVelocity(IntakeSpeeds.IntakeCoral.value())).until(()->intake.hasCoral());
    }

    public Command scoreL1(){
        return score(Positions.L1);
    }
    public Command presetL1(){
        return preset(Positions.L1);
    }

    public Command scoreL2(){
        return score(Positions.L2);
    }
    public Command presetL2(){
        return preset(Positions.L2);
    }

    public Command scoreL3(){
        return score(Positions.L3);
    }
    public Command presetL3(){
        return preset(Positions.L3);
    }

    public Command scoreL4(){
        return score(Positions.L4);
    }
    public Command presetL4(){
        return preset(Positions.L4);
    }

    public Command score(Positions position){
        return Commands.race(arm.reachGoalDegrees(position.armDegrees())
            ,elevator.reachGoal(position.elevatorMeters())
            ,Commands.waitSeconds(MAX_SIGNAL_TIME)
            .andThen(Commands.waitUntil(()->Math.abs(arm.getTarget()-arm.getPosition()) < MAX_ARM_ERROR 
                && Math.abs(elevator.getTargetMeters()-elevator.getPositionMeters()) < MAX_ELEVATOR_ERROR))
            .andThen(Commands.deadline(Commands.waitSeconds(SHOOT_TIME),
                intake.setVelocity(IntakeSpeeds.Shoot.value()))));
    }

    /**
     * does not end automatically
     * @param position
     * @return
     */
    public Command preset(Positions position){
        return Commands.parallel(arm.reachGoalDegrees(position.armDegrees())
            ,elevator.reachGoal(position.elevatorMeters()),
            intake.setVelocity(IntakeSpeeds.Idle.value()));
    }

    /**
     * does not end automatically, this slowly rises with 0 being 0% and 100% 
     * @param position
     * @return
     */
    public Command preset(Positions position, DoubleSupplier amount){
        return Commands.parallel(arm.reachGoalDegrees(position.armDegrees())
            ,elevator.reachGoal(()->position.elevatorMeters()*MathUtil.clamp(amount.getAsDouble(), 0, 1)),
            intake.setVelocity(IntakeSpeeds.Idle.value()));
    }

    public Command preset(int level, DoubleSupplier amount){
        if (level == 4){
            return preset(Positions.L4, amount);
        }
        if (level == 3){
            return preset(Positions.L3, amount);
        }
        if (level == 2){
            return preset(Positions.L2, amount);
        }
        return preset(Positions.L1, amount);
    }

    public Command score(int level){
        if (level == 4){
            return scoreL4();
        }
        if (level == 3){
            return scoreL3();
        }
        if (level == 2){
            return scoreL2();
        }
        return scoreL1();
    }

    public Command preset(int level){
        if (level == 4){
            return presetL4();
        }
        if (level == 3){
            return presetL3();
        }
        if (level == 2){
            return presetL2();
        }
        return presetL1();
    }

    public Command presetAlgaeUpper(){
        return preset(Positions.AlgaeU);
    }

    /**
     * ends after you get a piece
     * @return
     */
    public Command grabAlgae(Positions level){
        return Commands.parallel(arm.reachGoalDegrees(level.armDegrees()),
            elevator.reachGoal(level.elevatorMeters()),
            intake.setVelocity(IntakeSpeeds.IntakeAlgae.value())).until(()->intake.hasAlgae());
    }

    /**
     * ends after you get a piece
     * @return
     */
    public Command grabAlgae(int level){
        level = MathUtil.clamp(level, 1, 2);
        if (level == 1){
            return grabAlgae(Positions.AlgaeL);
        }else{
            return grabAlgae(Positions.AlgaeU);
        }
    }

    /**
     * ends after you get a piece
     * @return
     */
    public Command grabAlgaeUpper(){
        return grabAlgae(Positions.AlgaeU);
    }

    public Command presetAlgaeLower(){
        return preset(Positions.AlgaeL);
    }
    /**
     * ends after you get a piece
     * @return
     */
    public Command grabAlgaeLower(){
        return grabAlgae(Positions.AlgaeL);
    }

    public Command presetAlgaeNet(){
        return preset(Positions.AlgaeN);
    }
    public Command scoreAlgaeNet(){
        return score(Positions.AlgaeN);
    }

    public Command presetAlgaeProcessor(){
        return preset(Positions.AlgaeP);
    }
    public Command scoreAlgaeProcessor(){
        return score(Positions.AlgaeP);
    }
}
