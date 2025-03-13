package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import org.ironmaple.simulation.IntakeSimulation.IntakeSide;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.networktables.StructSubscriber;
import edu.wpi.first.units.measure.Per;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import frc.robot.Robot;
import frc.robot.constants.ArmConstants;
import frc.robot.constants.ElevatorConstants;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.ramp.Ramp;
import frc.robot.util.MapleSimWorld;

/**
 * every call pertaining to main scoring subsystems should be called through here
 */
public class MainMechanism {

    private final Notifier notifier;

    private final double MAX_ARM_ERROR = .05;
    private final double MAX_ELEVATOR_ERROR = .05;
    private final double MAX_RAMP_ERROR = .05;
    private final double MAX_SIGNAL_TIME = .03;
    private final double SHOOT_TIME = .3;
    public static final double ELEVATOR_END_DEFFECTOR_OFFSET = .6;

    public final Arm arm;

    public final Intake intake;

    public final Elevator elevator;

    public final Ramp ramp;

    private final NetworkTable robot = NetworkTableInstance.getDefault().getTable("Robot");
    private final NetworkTable armTable = robot.getSubTable("Arm");

    private final StructSubscriber<Pose3d> armSubscriber = armTable.getStructTopic("ArmAngle",Pose3d.struct).subscribe(new Pose3d());

    private final StructPublisher<Pose3d> coralPublisher = armTable.getStructTopic("CoralIfHad",Pose3d.struct).publish();

    private final NetworkTable odom = robot.getSubTable("Odometry");
    private StructSubscriber<Pose2d> poseSubscriber = odom
        .getStructTopic("RobotPose",Pose2d.struct).subscribe(new Pose2d());

    private final double rampUpPosition = -.25;
    private final double rampDownPosition = 0;
    private final double rampJigglePosition = -.04;

    public static enum Positions{
        Intake(-160,0),
        L1(0,.1),
        L2(-35,.5),
        L3(-35,1),
        L4(0,1.5),
        AlgaeU(45,.6),
        AlgaeL(45,.3),
        AlgaeN(-20,2),
        AlgaeP(20,0);

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

    public static enum IntakeSpeeds{
        Idle(0),
        IntakeCoral(7.5),
        IntakeAlgae(-30),
        HoldAlgae(-10),
        Shoot(30);

        private double velocity;
        IntakeSpeeds(double velocityRPS){
            this.velocity = velocityRPS;
        }

        public double value(){
            return velocity;
        }
    }

    public MainMechanism(Arm arm, Intake intake, Elevator elevator, Ramp ramp){
        notifier = new Notifier(this::periodic);
        notifier.startPeriodic(0.02);

        this.arm = arm;
        this.intake = intake;
        this.elevator = elevator;
        this.ramp = ramp;
        
        arm.setDefaultCommand(arm.reachGoalDegrees(()->{
            if (elevator.getPosition() < .2){
                return Positions.Intake.armDegrees();
            }
            return Positions.L4.armDegrees();
        }));
        intake.setDefaultCommand(intake.setVelocity(()->intake.hasAlgae() ? IntakeSpeeds.HoldAlgae.value() : 0));
        elevator.setDefaultCommand(elevator.reachGoal(IntakeSpeeds.Idle.value()));
        ramp.setDefaultCommand(ramp.reachGoal(rampUpPosition));
        if (Robot.isSimulation()){
            poseSubscriber = NetworkTableInstance.getDefault().getTable("RealData")
                .getStructTopic("RealPose", Pose2d.struct).subscribe(new Pose2d());
            MapleSimWorld.addIntakeSimulation("CoralIntake", "Coral", .5,.4, new Translation2d(-.1,0));
            // MapleSimWorld.addShooterSimulation(()->
            //     new Transform3d(0.2 + (arm.getPosition() > 1.0/7.0 ? .29 : 0),0, elevator.getPositionMeters() + ELEVATOR_END_DEFFECTOR_OFFSET,new Rotation3d(0, (arm.getPosition() > 1.0/7.0 ? -Math.PI/2 : -Units.rotationsToRadians(arm.getPosition())),0))
            //     , ()->2
            //     , "Coral"
            //     , "CoralIntake");
            MapleSimWorld.addShooterSimulation(()->{
                Pose3d armPose = armSubscriber.get(new Pose3d());
                double yOffset = ArmConstants.ARM_LENGTH_METERS * Math.cos(Units.rotationsToRadians(ArmConstants.ARM_END_DEFFECTOR_SCORE_OFFSET) + armPose.getRotation().getY());
                double xOffset = ArmConstants.ARM_LENGTH_METERS * Math.sin(Units.rotationsToRadians(ArmConstants.ARM_END_DEFFECTOR_SCORE_OFFSET) + armPose.getRotation().getY());
                return new Transform3d(armPose.getX() + xOffset,0, armPose.getZ() + yOffset
                    ,new Rotation3d(0, -armPose.getRotation().getY() - Units.rotationsToRadians(ArmConstants.ARM_END_DEFFECTOR_SCORE_ANGLE),0));
                }
                
                , ()->2
                , "Coral"
                , "CoralIntake");
            MapleSimWorld.addIntakeRequirements("CoralIntake", ()->intake.getVelocity() > 5);
            MapleSimWorld.addIntakeRequirements("CoralIntake", ()->Math.abs(arm.getPosition() - Positions.Intake.armPosition) < .1);
            MapleSimWorld.addIntakeRequirements("CoralIntake", ()->Math.abs(elevator.getPosition() - Positions.Intake.elevatorMeters) < .1);
            MapleSimWorld.hasPiece("CoralIntake",(has)->intake.getCoralDigitalInputIO().setValue(has));
            MapleSimWorld.addShootRequirements("CoralIntake", ()->intake.getVelocity() > 10);

            // MapleSimWorld.addIntakeSimulation("AlgaeIntake","Algae", .5,.4,new Translation2d(.3,0));
            // MapleSimWorld.addIntakeRequirements("AlgaeIntake", ()->intake.getVelocity() < -20);
            // MapleSimWorld.hasPiece("AlgaeIntake",(has)->intake.getAlgaeDigitalInputIO().setValue(has));

            // MapleSimWorld.addShooterSimulation(()->
            //     new Transform3d(0.2,0, elevator.getPositionMeters() + ELEVATOR_END_DEFFECTOR_OFFSET,new Rotation3d(0, -Units.rotationsToRadians(arm.getPosition()),0))
            //     , ()->2
            //     , "Algae"
            //     , "AlgaeIntake");
            // MapleSimWorld.addShootRequirements("AlgaeIntake", ()->intake.getVelocity() > 10);


        }
        Runtime.getRuntime().addShutdownHook(new Thread(notifier::close));
    }

    /**
     * keep in mind, toState ends automatically!!!
     * @param position
     * @return
     */
    private Command toState(Positions position){
        return arm.reachGoalDegrees(Positions.L4.armDegrees()) // Goes to the other side of the elevator top bar
            .until(()->Math.abs(arm.getTarget()-arm.getPosition()) < MAX_ARM_ERROR)
            .andThen(elevator.reachGoal(position.elevatorMeters()))
            .until(()->Math.abs(position.elevatorMeters()-elevator.getPosition()) < MAX_ELEVATOR_ERROR)
            .andThen(arm.reachGoalDegreesOnce(position.armDegrees()));
    }

    public Command idle(){
        return toState(Positions.Intake)
            .alongWith(intake.setVelocity(IntakeSpeeds.Idle.value()));
    }

    /**
     * ends after you get a piece
     * @return
     */
    public Command intake(){
        return Commands.parallel(
            toState(Positions.Intake)
            ,intake.setVelocity(IntakeSpeeds.IntakeCoral.value())
            ,jiggleRamp()).until(()->intake.hasCoral())
            .andThen(Commands.deadline(intake.stop(), ramp.reachGoal(rampUpPosition)));
    }

    public Command jiggleRamp(){
        return (ramp.reachGoal(rampDownPosition)
            .until(()->Math.abs(ramp.getTarget() - ramp.getPosition()) < MAX_RAMP_ERROR)
            .andThen(ramp.reachGoal(rampJigglePosition)
                .until(()->Math.abs(ramp.getTarget() - ramp.getPosition()) < MAX_RAMP_ERROR))).repeatedly();
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
        return toState(position).deadlineFor(ramp.reachGoal(rampUpPosition))
            .andThen(Commands.waitUntil(()->Math.abs(Units.degreesToRotations(position.armDegrees())-arm.getPosition()) < MAX_ARM_ERROR 
                && Math.abs(elevator.getTarget()-elevator.getPosition()) < MAX_ELEVATOR_ERROR))
            .andThen(intake.setVelocity(IntakeSpeeds.Shoot.value()).withTimeout(SHOOT_TIME));
    }

    /**
     * does not end automatically
     * @param position
     * @return
     */
    public Command preset(Positions position){
        return preset(position, ()->1);
    }

    /**
     * does not end automatically, this slowly rises with 0 being 0% and 100% 
     * @param position
     * @return
     */
    public Command preset(Positions position, DoubleSupplier amount){
        return Commands.deadline(arm.reachGoalDegrees(Positions.L4.armDegrees())
            .until(()->Math.abs(Units.rotationsToDegrees(arm.getPosition())-Positions.L4.armDegrees()) < Units.rotationsToDegrees(MAX_ARM_ERROR))
            .andThen(elevator.reachGoal(()->position.elevatorMeters()*MathUtil.clamp(amount.getAsDouble(), 0, 1))).deadlineFor(ramp.reachGoal(rampUpPosition))
            .until(()->Math.abs(position.elevatorMeters()-elevator.getPosition()) < MAX_ELEVATOR_ERROR)
            .andThen(arm.reachGoalDegrees(position.armDegrees())));
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
        switch (level) {
            case 4:
                return scoreL4();
            case 3:
                return scoreL3();
            case 2:
                return scoreL2();        
            default:
                return scoreL1();
        }
    }

    public Command preset(int level){
        switch (level) {
            case 4:
                return presetL4();
            case 3:
                return presetL3();
            case 2:
                return presetL2();        
            default:
                return presetL1();
        }
    }

    public Command presetAlgaeUpper(){
        return preset(Positions.AlgaeU);
    }

    /**
     * ends after you get a piece
     * @return
     */
    public Command grabAlgae(Positions level){
        return Commands.parallel(toState(level),
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
    public Command scoreAlgae(int level){
        level = MathUtil.clamp(level, 1, 2);
        if (level == 1){
            return scoreAlgaeProcessor();
        }else{
            return scoreAlgaeNet();
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

    public Command testPosition(DoubleSupplier elevatorMeters, DoubleSupplier armDegrees, DoubleSupplier intakeSpeed){
        return elevator.reachGoal(elevatorMeters).alongWith(arm.reachGoalDegrees(armDegrees)).alongWith(intake.setVelocity(intakeSpeed));
    }

    public void periodic(){
        if (intake.hasCoral()){
            Pose3d armPose = armSubscriber.get(new Pose3d());
                double yOffset = ArmConstants.ARM_LENGTH_METERS * Math.cos(Units.rotationsToRadians(ArmConstants.ARM_END_DEFFECTOR_SCORE_OFFSET) + armPose.getRotation().getY());
                double xOffset = ArmConstants.ARM_LENGTH_METERS * Math.sin(Units.rotationsToRadians(ArmConstants.ARM_END_DEFFECTOR_SCORE_OFFSET) + armPose.getRotation().getY());
            Transform3d t = new Transform3d(armPose.getX() + xOffset,0, armPose.getZ() + yOffset
            ,new Rotation3d(0, armPose.getRotation().getY() + Units.rotationsToRadians(ArmConstants.ARM_END_DEFFECTOR_SCORE_ANGLE),0));
            coralPublisher.accept(new Pose3d(poseSubscriber.get()).plus(t));
        }else{
            coralPublisher.accept(new Pose3d());
        }
    }
}
