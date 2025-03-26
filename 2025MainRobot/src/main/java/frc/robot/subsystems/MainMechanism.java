package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amp;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.ironmaple.simulation.IntakeSimulation.IntakeSide;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.networktables.StructSubscriber;
import edu.wpi.first.units.measure.Per;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import frc.robot.Robot;
import frc.robot.constants.ArmConstants;
import frc.robot.constants.ElevatorConstants;
import frc.robot.constants.GameData;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.ramp.Ramp;
import frc.robot.util.MapleSimWorld;
import frc.robot.util.PoseEX;

// /**
//  * every call pertaining to main scoring subsystems should be called through here
//  */
// public class MainMechanism {

//     private final double MAX_ARM_ERROR = .1;
//     private final double MAX_ELEVATOR_ERROR = .1;
//     private final double MAX_RAMP_ERROR = .05;
//     private final double MAX_SIGNAL_TIME = .03;
//     private final double SHOOT_TIME = .4;
//     private final double INTAKE_TIME = .05;
//     public static final double ELEVATOR_END_DEFFECTOR_OFFSET = .6;

//     private final Notifier notifier;

//     public final Elevator elevator;
//     public final Arm arm;
//     public final Intake intake;
//     public final Ramp ramp;

//     private final NetworkTable robot = NetworkTableInstance.getDefault().getTable("Robot");
//     private final NetworkTable armTable = robot.getSubTable("Arm");

//     private final StructSubscriber<Pose3d> armSubscriber = armTable.getStructTopic("ArmAngle",Pose3d.struct).subscribe(new Pose3d());

//     private final StructPublisher<Pose3d> coralPublisher = armTable.getStructTopic("CoralIfHad",Pose3d.struct).publish();

//     private final NetworkTable odom = robot.getSubTable("Odometry");
//     private StructSubscriber<Pose2d> poseSubscriber = odom
//         .getStructTopic("RobotPose",Pose2d.struct).subscribe(new Pose2d());

//     public static enum Positions{
        
//         CoralSafe(-0.27,0.05, false),
//         CoralReset(-0.224609,0.05, false),
//         Idle(-.422871,0.05, false),
//         Intake(-.405,0, false),
//         L1(-0.30625,.1, false),
//         L2(-0.30625,.55, false),
//         L3(-0.30625,1.44, false),
//         L4(-0.224609,3, false),
//         AlgaeReset(-.2,0, true),
//         AlgaeU(0,1.8,true),
//         AlgaeL(0,1, true),
//         AlgaeN(0-.1,3.5, true),
//         AlgaeP(0,0, true);

//         private final double armRotations;
//         private final double elevatorMeters;
//         private final boolean algaeSafe;

//         Positions(double armRotations, double elevatorMeters){
//             this.armRotations = armRotations;
//             this.elevatorMeters = elevatorMeters;
//             this.algaeSafe = false;
//         }

//         Positions(double armRotations, double elevatorMeters, boolean algaeSafe){
//             this.armRotations = armRotations;
//             this.elevatorMeters = elevatorMeters;
//             this.algaeSafe = algaeSafe;
//         }

//         public double armRotations(){
//             return armRotations;
//         }

//         public double elevatorMeters(){
//             return elevatorMeters;
//         }

//         public boolean algaeSafe(){
//             return algaeSafe;
//         }
//     }

//     public static enum IntakeSpeeds{
//         Off(0),
//         IntakeCoral(30),
//         ShootCoral(60),
//         IntakeAlgae(-60),
//         HoldAlgae(-60),
//         ShootAlgae(60);

//         private final double velocity;
//         IntakeSpeeds(double velocity){
//             this.velocity = velocity;
//         }

//         public double getVelocity(){
//             return velocity;
//         }
//     }

//     public static enum RampPositions{
//         Up(-.186),
//         Down(-.6),
//         Jiggle(-.2);

//         private final double rotation;
//         RampPositions(double rotation){
//             this.rotation = rotation;
//         }

//         public double getPosition(){
//             return rotation;
//         }
//     }

//     public MainMechanism(Arm arm, Intake intake, Elevator elevator, Ramp ramp){
//         this.elevator = elevator;
//         this.arm = arm;
//         this.intake = intake;
//         this.ramp = ramp;

//         elevator.setDefaultCommand(elevator.setVoltage(-1));
//         arm.setDefaultCommand(arm.reachGoal(()->
//             elevator.getPosition() > .2 ? intake.hasAlgae() ? Positions.AlgaeReset.armRotations() : Positions.L4.armRotations() : intake.hasAlgae() ? Positions.AlgaeReset.armRotations() : Positions.Intake.armRotations()
//         ));
//         intake.setDefaultCommand(intake.setVelocity(()->intake.hasAlgae() ? IntakeSpeeds.HoldAlgae.getVelocity() : IntakeSpeeds.Off.getVelocity()));

//         if (Robot.isSimulation()){
//             poseSubscriber = NetworkTableInstance.getDefault().getTable("RealData")
//                 .getStructTopic("RealPose", Pose2d.struct).subscribe(new Pose2d());
//             MapleSimWorld.addIntakeSimulation("CoralIntake", "Coral", .5,.4, new Translation2d(-.1,0));
//             // MapleSimWorld.addShooterSimulation(()->
//             //     new Transform3d(0.2 + (arm.getPosition() > 1.0/7.0 ? .29 : 0),0, elevator.getPositionMeters() + ELEVATOR_END_DEFFECTOR_OFFSET,new Rotation3d(0, (arm.getPosition() > 1.0/7.0 ? -Math.PI/2 : -Units.rotationsToRadians(arm.getPosition())),0))
//             //     , ()->2
//             //     , "Coral"
//             //     , "CoralIntake");
//             MapleSimWorld.addShooterSimulation(()->{
//                     Pose3d armPose = armSubscriber.get(new Pose3d());
//                     double yOffset = ArmConstants.ARM_LENGTH_METERS * Math.cos(Units.rotationsToRadians(ArmConstants.ARM_END_DEFFECTOR_SCORE_OFFSET) + armPose.getRotation().getY());
//                     double xOffset = ArmConstants.ARM_LENGTH_METERS * Math.sin(Units.rotationsToRadians(ArmConstants.ARM_END_DEFFECTOR_SCORE_OFFSET) + armPose.getRotation().getY());
//                         return new Transform3d(armPose.getX() + xOffset-.05,0, armPose.getZ() + yOffset
//                         ,new Rotation3d(0, armPose.getRotation().getY() + Units.rotationsToRadians(ArmConstants.ARM_END_DEFFECTOR_SCORE_ANGLE)+Math.PI,armSubscriber.get().getZ() > 1.5 ? 0 : Math.PI));
//                 }
//                 // Pose3d armPose = armSubscriber.get(new Pose3d());
//                 // double yOffset = ArmConstants.ARM_LENGTH_METERS * Math.cos(Units.rotationsToRadians(ArmConstants.ARM_END_DEFFECTOR_SCORE_OFFSET) + armPose.getRotation().getY());
//                 // double xOffset = ArmConstants.ARM_LENGTH_METERS * Math.sin(Units.rotationsToRadians(ArmConstants.ARM_END_DEFFECTOR_SCORE_OFFSET) + armPose.getRotation().getY());
//                 // return new Transform3d(armPose.getX() + xOffset,0, armPose.getZ() + yOffset
//                 //     ,new Rotation3d(0, armPose.getRotation().getY() + Units.rotationsToRadians(ArmConstants.ARM_END_DEFFECTOR_SCORE_ANGLE),0));
//                 // }
                
//                 , ()->2
//                 , "Coral"
//                 , "CoralIntake");
//             MapleSimWorld.addIntakeRequirements("CoralIntake", ()->intake.getVelocity() > 10);
//             MapleSimWorld.addIntakeRequirements("CoralIntake", ()->Math.abs(arm.getPosition() - Positions.Intake.armRotations()) < .1);
//             MapleSimWorld.addIntakeRequirements("CoralIntake", ()->Math.abs(elevator.getPosition() - Positions.Intake.elevatorMeters) < .1);
//             MapleSimWorld.hasPiece("CoralIntake",(has)->intake.getCoralDigitalInputIO().setValue(has));
//             MapleSimWorld.addShootRequirements("CoralIntake", ()->intake.getVelocity() > 31);

//             MapleSimWorld.addIntakeSimulation("AlgaeIntake","Algae", .5,.4,new Translation2d(.3,0));
//             MapleSimWorld.addIntakeRequirements("AlgaeIntake", ()->intake.getVelocity() < -20);
//             MapleSimWorld.hasPiece("AlgaeIntake",(has)->intake.getAlgaeDigitalInputIO().setValue(has));

//             MapleSimWorld.addShooterSimulation(()->
//                 new Transform3d(0.2,0, elevator.getPosition() + ELEVATOR_END_DEFFECTOR_OFFSET,new Rotation3d(0, -Units.rotationsToRadians(arm.getPosition()),0))
//                 , ()->2
//                 , "Algae"
//                 , "AlgaeIntake");
//             MapleSimWorld.addShootRequirements("AlgaeIntake", ()->intake.getVelocity() > 10);
//         }

//         notifier = new Notifier(this :: periodic);
//         notifier.setName("Scoring Periodic");
//         notifier.startPeriodic(.02);
//         Runtime.getRuntime().addShutdownHook(new Thread(notifier::close));
//     }

//     /**
//      * ends when it has made each subsytem act in the wanted manner, purely goes to a position without any hits. (A preset)
//      * @param positions
//      * @return
//      */
//     public Command toState(Positions positions, DoubleSupplier amount, boolean ending){
//         return Commands.either(Commands.either(toSafeState(positions, Positions.AlgaeReset, amount, ending)
//                 , Commands.none()
//                 , ()->positions.algaeSafe())
//             , toSafeState(positions, Positions.CoralReset, amount, ending)
//             , ()->intake.hasAlgae());
//     }

//     /**
//      * ends when it has made each subsytem act in the wanted manner, purely goes to a position without any hits. (A preset)
//      * @param positions
//      * @return
//      */
//     public Command toState(Positions positions){
//         return toState(positions, ()->1, true);
//     }

//     //TODO this is an old, but stable version, the other is new, but may work, and be much better for it
//     // private Command toSafeState(Positions positions, Positions safe, DoubleSupplier amount, boolean ending){
//     //     return arm.reachGoal(safe.armRotations()).until(()->Math.abs(arm.getTarget() - arm.getPosition()) < MAX_ARM_ERROR)
//     //         .andThen(elevator.reachGoal(()->positions.elevatorMeters()*MathUtil.clamp(amount.getAsDouble(),0,1))).until(()->Math.abs(positions.elevatorMeters() - elevator.getPosition()) < MAX_ELEVATOR_ERROR)
//     //         .andThen(Commands.either(
//     //             arm.reachGoalOnce(positions.armRotations())
//     //             , elevator.reachGoal(()->positions.elevatorMeters()*MathUtil.clamp(amount.getAsDouble(),0,1))
//     //                 .alongWith(arm.reachGoal(()->Math.abs(positions.elevatorMeters() - elevator.getPosition()) < MAX_ELEVATOR_ERROR ? positions.armRotations() : safe.armRotations()))
//     //             , ()->ending));
//     // }

//     private Command toSafeState(Positions positions, Positions safe, DoubleSupplier amount, boolean ending){
//         return arm.reachGoal(safe.armRotations()).until(()->arm.getPosition() > Positions.CoralSafe.armRotations())
//             .andThen(elevator.reachGoal(()->positions.elevatorMeters()*MathUtil.clamp(amount.getAsDouble(),0,1))).until(()->Math.abs(positions.elevatorMeters() - elevator.getPosition()) < MAX_ELEVATOR_ERROR)
//             .andThen(Commands.either(
//                 arm.reachGoalOnce(positions.armRotations())
//                 , elevator.reachGoal(()->positions.elevatorMeters()*MathUtil.clamp(amount.getAsDouble(),0,1))
//                     .alongWith(arm.reachGoal(()->Math.abs(positions.elevatorMeters() - elevator.getPosition()) < MAX_ELEVATOR_ERROR ? positions.armRotations() : safe.armRotations()))
//                 , ()->ending));
//     }

//     public Command resetElevator(){
//         return elevator.reachGoalOnce(Positions.Intake.elevatorMeters());
//     }

//     public Command preset(Positions positions, DoubleSupplier amount){
//         return toState(positions, amount, false);
//     }

//     public Command preset(Positions positions){
//         return preset(positions, ()->1);
//     }
    
//     public Command presetCoral(int level, DoubleSupplier amount){
//         switch (level) {
//             case 1:
//                 return preset(Positions.L1, amount);
//             case 2:
//                 return preset(Positions.L2, amount);
//             case 3:
//                 return preset(Positions.L3, amount);
//             case 4:
//                 return preset(Positions.L4, amount);
        
//             default:
//                 return preset(Positions.Intake);
//         }
//     }

//     public Command presetCoral(int level){
//         return presetCoral(level, ()->1);
//     }

//     public Command presetAlgae(int level, DoubleSupplier amount){
//         switch (level) {
//             case 1:
//                 return preset(Positions.AlgaeP, amount);
//             case 2:
//                 return preset(Positions.AlgaeN, amount);
        
//             default:
//                 return preset(Positions.Intake, amount);
//         }
//     }

//     public Command presetAlgae(int level){
//         return presetAlgae(level, ()->1);
//     }

//     public Command presetAlgaeIntake(int level, DoubleSupplier amount){
//         switch (level) {
//             case 1:
//                 return preset(Positions.AlgaeL, amount);
//             case 2:
//                 return preset(Positions.AlgaeU, amount);
        
//             default:
//                 return preset(Positions.Intake, amount);
//         }
//     }

//     public Command presetAlgaeIntake(int level){
//         return presetAlgae(level, ()->1);
//     }

//     public Command intake(){
//         return Commands.either(Commands.none()
//             ,Commands.parallel(toState(Positions.Intake)
//             ,intake.setVelocity(IntakeSpeeds.IntakeCoral.getVelocity())
//             ,jiggleRamp())
//             .until(()->intake.hasCoral())
//             .andThen(Commands.waitSeconds(INTAKE_TIME))
//             .andThen(intake.stop())
//             , ()->intake.hasAlgae());
//     }

//     public Command jiggleRamp(){
//         return (ramp.reachGoal(RampPositions.Up.getPosition())
//             .until(()->Math.abs(ramp.getTarget() - ramp.getPosition()) < .01)
//             .andThen(ramp.reachGoal(RampPositions.Up.getPosition())
//                 .until(()->Math.abs(ramp.getTarget() - ramp.getPosition()) < .01))).repeatedly();
//     }

//     public Command score(Positions positions, IntakeSpeeds speed){
//         return toState(positions).andThen(intake.setVelocity(speed.getVelocity()).withTimeout(SHOOT_TIME));
//     }

//     public Command scoreCoral(int level){
//         switch (level) {
//             case 1:
//                 return score(Positions.L1, IntakeSpeeds.ShootCoral);
//             case 2:
//                 return score(Positions.L2, IntakeSpeeds.ShootCoral);
//             case 3:
//                 return score(Positions.L3, IntakeSpeeds.ShootCoral);
//             case 4:
//                 return score(Positions.L4, IntakeSpeeds.ShootCoral);
        
//             default:
//                 return preset(Positions.Intake);
//         }
//     }

//     public Command scoreAlgae(int level){
//         switch (level) {
//             case 1:
//                 return score(Positions.AlgaeP, IntakeSpeeds.ShootAlgae);
//             case 2:
//                 return score(Positions.AlgaeN, IntakeSpeeds.ShootAlgae);
        
//             default:
//                 return preset(Positions.Intake);
//         }
//     }

//     public Command intakeAlgae(Positions positions){
//         return Commands.either(Commands.none()
//             ,Commands.parallel(toState(positions)
//             ,intake.setVelocity(IntakeSpeeds.IntakeAlgae.getVelocity()))
//             .until(()->intake.hasAlgae())
//             .andThen(intake.setVelocity(IntakeSpeeds.HoldAlgae.getVelocity()))
//             , ()->intake.hasCoral());
//     }

//     public Command intakeAlgae(int level){
//         switch (level) {
//             case 1:
//                 return intakeAlgae(Positions.AlgaeL);
//             case 2:
//                 return intakeAlgae(Positions.AlgaeU);
        
//             default:
//                 return preset(Positions.Intake);
//         }
//     }

//     public Command testPosition(DoubleSupplier elevatorMeters, DoubleSupplier armRotations, DoubleSupplier intakeSpeed){
//         return elevator.reachGoal(elevatorMeters).alongWith(arm.reachGoal(armRotations)).alongWith(intake.setVelocity(intakeSpeed));
//     }

//     public void periodic(){
//         if (intake.hasCoral()){
//             Pose3d armPose = armSubscriber.get(new Pose3d());
//                 double yOffset = ArmConstants.ARM_LENGTH_METERS * Math.cos(Units.rotationsToRadians(ArmConstants.ARM_END_DEFFECTOR_SCORE_OFFSET) + armPose.getRotation().getY());
//                 double xOffset = ArmConstants.ARM_LENGTH_METERS * Math.sin(Units.rotationsToRadians(ArmConstants.ARM_END_DEFFECTOR_SCORE_OFFSET) + armPose.getRotation().getY());
//             Transform3d t = new Transform3d(armPose.getX() + xOffset,0, armPose.getZ() + yOffset
//             ,new Rotation3d(0, armPose.getRotation().getY() + Units.rotationsToRadians(ArmConstants.ARM_END_DEFFECTOR_SCORE_ANGLE),0));
//             coralPublisher.accept(new Pose3d(poseSubscriber.get()).plus(t));
//         }else{
//             coralPublisher.accept(new Pose3d());
//         }
//     }
// }

/**
 * every call pertaining to main scoring subsystems should be called through here
 */
public class MainMechanism {

    private final double MAX_ARM_ERROR = .1;
    private final double MAX_ELEVATOR_ERROR = .1;
    private final double MAX_RAMP_ERROR = .05;
    private final double MAX_SIGNAL_TIME = .03;
    private final double SHOOT_TIME = .3;
    private final double INTAKE_TIME = .05;
    public static final double ELEVATOR_END_DEFFECTOR_OFFSET = .6;

    private final Notifier notifier;

    public final Elevator elevator;
    public final Arm arm;
    public final Intake intake;
    public final Ramp ramp;

    private final NetworkTable robot = NetworkTableInstance.getDefault().getTable("Robot");
    private final NetworkTable armTable = robot.getSubTable("Arm");
    private final NetworkTable elevTable = robot.getSubTable("Elevator");

    private final StructSubscriber<Pose3d> armSubscriber = armTable.getStructTopic("ArmAngle",Pose3d.struct).subscribe(new Pose3d());

    private final StructPublisher<Pose3d> coralPublisher = armTable.getStructTopic("CoralIfHad",Pose3d.struct).publish();

    private final BooleanPublisher resetPublisher = elevTable.getBooleanTopic("ElevatorNeedsReset").publish();

    private final NetworkTable odom = robot.getSubTable("Odometry");
    private StructSubscriber<Pose2d> poseSubscriber = odom
        .getStructTopic("RobotPose",Pose2d.struct).subscribe(new Pose2d());

    private final Supplier<Pose2d> robotPose;

    private boolean elevatorNeedsReset = false;

    public static enum Positions{
        
        CoralSafe(-0.27,0.05, false),
        CoralReset(-0.224609,0.05, false),
        Idle(-.422871,0.05, false),
        Intake(-.405,0, false),
        L1(-0.30625,.1, false),
        L2(-0.30625,.55, false),
        L3(-0.30625,1.44, false),
        L4(-0.224609,2.95, false),
        AlgaeReset(0,0, true),
        AlgaeU(0,1.8,true),
        AlgaeL(0,1, true),
        AlgaeN(0-.1,3.5, true),
        AlgaeP(0,0, true);

        private final double armRotations;
        private final double elevatorMeters;
        private final boolean algaeSafe;

        Positions(double armRotations, double elevatorMeters){
            this.armRotations = armRotations;
            this.elevatorMeters = elevatorMeters;
            this.algaeSafe = false;
        }

        Positions(double armRotations, double elevatorMeters, boolean algaeSafe){
            this.armRotations = armRotations;
            this.elevatorMeters = elevatorMeters;
            this.algaeSafe = algaeSafe;
        }

        public double armRotations(){
            return armRotations;
        }

        public double elevatorMeters(){
            return elevatorMeters;
        }

        public boolean algaeSafe(){
            return algaeSafe;
        }
    }

    public static enum IntakeSpeeds{
        Off(0),
        IntakeCoral(30),
        ShootCoral(60),
        IntakeAlgae(-60),
        HoldAlgae(-60),
        ShootAlgae(60);

        private final double velocity;
        IntakeSpeeds(double velocity){
            this.velocity = velocity;
        }

        public double getVelocity(){
            return velocity;
        }
    }

    public static enum RampPositions{
        Up(-.186),
        Down(-.6),
        Jiggle(-.2);

        private final double rotation;
        RampPositions(double rotation){
            this.rotation = rotation;
        }

        public double getPosition(){
            return rotation;
        }
    }

    public MainMechanism(Arm arm, Intake intake, Elevator elevator, Ramp ramp, Supplier<Pose2d> robotPose){
        this.robotPose = robotPose;
        this.elevator = elevator;
        this.arm = arm;
        this.intake = intake;
        this.ramp = ramp;

        elevator.setDefaultCommand(
            elevator.run(()->{
                boolean a = PoseEX.getDistanceFromPoseMeters(robotPose.get(),
                GameData.reefCenterPose(DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red)) > 1.5;
                if (a){
                    elevator.setPosition(0); 
                }else{ 
                    elevator.setVolts(0);
                }
        }));

        arm.setDefaultCommand(arm.reachGoal(()->
            intake.hasAlgae() ? Positions.AlgaeP.armRotations() : elevator.getPosition() > .2 ? Positions.L4.armRotations() : Positions.Intake.armRotations()
        ));
        intake.setDefaultCommand(intake.setVelocity(()->intake.hasAlgae() ? IntakeSpeeds.HoldAlgae.getVelocity() : IntakeSpeeds.Off.getVelocity()));

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
                        return new Transform3d(armPose.getX() + xOffset-.05,0, armPose.getZ() + yOffset
                        ,new Rotation3d(0, armPose.getRotation().getY() + Units.rotationsToRadians(ArmConstants.ARM_END_DEFFECTOR_SCORE_ANGLE)+Math.PI,armSubscriber.get().getZ() > 1.5 ? 0 : Math.PI));
                }
                // Pose3d armPose = armSubscriber.get(new Pose3d());
                // double yOffset = ArmConstants.ARM_LENGTH_METERS * Math.cos(Units.rotationsToRadians(ArmConstants.ARM_END_DEFFECTOR_SCORE_OFFSET) + armPose.getRotation().getY());
                // double xOffset = ArmConstants.ARM_LENGTH_METERS * Math.sin(Units.rotationsToRadians(ArmConstants.ARM_END_DEFFECTOR_SCORE_OFFSET) + armPose.getRotation().getY());
                // return new Transform3d(armPose.getX() + xOffset,0, armPose.getZ() + yOffset
                //     ,new Rotation3d(0, armPose.getRotation().getY() + Units.rotationsToRadians(ArmConstants.ARM_END_DEFFECTOR_SCORE_ANGLE),0));
                // }
                
                , ()->2
                , "Coral"
                , "CoralIntake");
            MapleSimWorld.addIntakeRequirements("CoralIntake", ()->intake.getVelocity() > 10);
            MapleSimWorld.addIntakeRequirements("CoralIntake", ()->Math.abs(arm.getPosition() - Positions.Intake.armRotations()) < .1);
            MapleSimWorld.addIntakeRequirements("CoralIntake", ()->Math.abs(elevator.getPosition() - Positions.Intake.elevatorMeters) < .1);
            MapleSimWorld.hasPiece("CoralIntake",(has)->intake.getCoralDigitalInputIO().setValue(has));
            MapleSimWorld.addShootRequirements("CoralIntake", ()->intake.getVelocity() > 31);

            MapleSimWorld.addIntakeSimulation("AlgaeIntake","Algae", .5,.4,new Translation2d(.3,0));
            MapleSimWorld.addIntakeRequirements("AlgaeIntake", ()->intake.getVelocity() < -20);
            MapleSimWorld.hasPiece("AlgaeIntake",(has)->intake.getAlgaeDigitalInputIO().setValue(has));

            MapleSimWorld.addShooterSimulation(()->
                new Transform3d(0.2,0, elevator.getPosition() + ELEVATOR_END_DEFFECTOR_OFFSET,new Rotation3d(0, -Units.rotationsToRadians(arm.getPosition()),0))
                , ()->2
                , "Algae"
                , "AlgaeIntake");
            MapleSimWorld.addShootRequirements("AlgaeIntake", ()->intake.getVelocity() > 10);
        }

        notifier = new Notifier(this :: periodic);
        notifier.setName("Scoring Periodic");
        notifier.startPeriodic(.02);
        Runtime.getRuntime().addShutdownHook(new Thread(notifier::close));
    }

    // /**
    //  * ends when it has made each subsytem act in the wanted manner, purely goes to a position without any hits. (A preset)
    //  * @param positions
    //  * @return
    //  */
    // public Command toState(Positions positions, DoubleSupplier amount, boolean ending){
    //     return Commands.either(Commands.either(toSafeState(positions, Positions.AlgaeReset, amount, ending)
    //             , Commands.none()
    //             , ()->positions.algaeSafe())
    //         , toSafeState(positions, Positions.CoralReset, amount, ending)
    //         , ()->intake.hasAlgae());
    // }

    public Command toState(Positions positions, DoubleSupplier amount, boolean ending){
        return Commands.either(toSafeState(positions, amount, ending), (arm.reachGoal(positions.armRotations())
            .alongWith(elevator.reachGoal(()->positions.elevatorMeters() * MathUtil.clamp(amount.getAsDouble(),0,1))))
                .until(()->ending && Math.abs(arm.getPosition() - positions.armRotations()) < MAX_ARM_ERROR
                    && Math.abs(elevator.getPosition() - positions.elevatorMeters()) < MAX_ELEVATOR_ERROR),()->elevator.getPosition() - positions.elevatorMeters() > .2);
    }

    /**
     * ends when it has made each subsytem act in the wanted manner, purely goes to a position without any hits. (A preset)
     * @param positions
     * @return
     */
    public Command toState(Positions positions){
        return toState(positions, ()->1, true);
    }

    //TODO this is an old, but stable version, the other is new, but may work, and be much better for it
    // private Command toSafeState(Positions positions, Positions safe, DoubleSupplier amount, boolean ending){
    //     return arm.reachGoal(safe.armRotations()).until(()->Math.abs(arm.getTarget() - arm.getPosition()) < MAX_ARM_ERROR)
    //         .andThen(elevator.reachGoal(()->positions.elevatorMeters()*MathUtil.clamp(amount.getAsDouble(),0,1))).until(()->Math.abs(positions.elevatorMeters() - elevator.getPosition()) < MAX_ELEVATOR_ERROR)
    //         .andThen(Commands.either(
    //             arm.reachGoalOnce(positions.armRotations())
    //             , elevator.reachGoal(()->positions.elevatorMeters()*MathUtil.clamp(amount.getAsDouble(),0,1))
    //                 .alongWith(arm.reachGoal(()->Math.abs(positions.elevatorMeters() - elevator.getPosition()) < MAX_ELEVATOR_ERROR ? positions.armRotations() : safe.armRotations()))
    //             , ()->ending));
    // }

    private Command toSafeState(Positions positions, DoubleSupplier amount, boolean ending){
        return Commands.either(arm.reachGoal(Positions.AlgaeP.armRotations()), arm.reachGoal(Positions.CoralReset.armRotations()), ()->intake.hasAlgae()).until(()->arm.getPosition() > Positions.CoralSafe.armRotations())
            .andThen(elevator.reachGoal(()->positions.elevatorMeters()*MathUtil.clamp(amount.getAsDouble(),0,1))).until(()->Math.abs(positions.elevatorMeters() - elevator.getPosition()) < MAX_ELEVATOR_ERROR)
            .andThen(Commands.either(
                arm.reachGoalOnce(positions.armRotations())
                , elevator.reachGoal(()->positions.elevatorMeters()*MathUtil.clamp(amount.getAsDouble(),0,1))
                    .alongWith(arm.reachGoal(()->Math.abs(positions.elevatorMeters() - elevator.getPosition()) < MAX_ELEVATOR_ERROR ? positions.armRotations() : Positions.CoralReset.armRotations()))
                , ()->ending));
    }

    public Command resetElevator(){
        return elevator.reachGoalOnce(Positions.Intake.elevatorMeters());
    }

    public Command preset(Positions positions, DoubleSupplier amount){
        return toState(positions, amount, false);
    }

    public Command preset(Positions positions){
        return preset(positions, ()->1);
    }
    
    public Command presetCoral(int level, DoubleSupplier amount){
        switch (level) {
            case 1:
                return preset(Positions.L1, amount);
            case 2:
                return preset(Positions.L2, amount);
            case 3:
                return preset(Positions.L3, amount);
            case 4:
                return preset(Positions.L4, amount);
        
            default:
                return preset(Positions.Intake);
        }
    }

    public Command presetCoral(int level){
        return presetCoral(level, ()->1);
    }

    public Command presetAlgae(int level, DoubleSupplier amount){
        switch (level) {
            case 1:
                return preset(Positions.AlgaeP, amount);
            case 2:
                return preset(Positions.AlgaeN, amount);
        
            default:
                return preset(Positions.Intake, amount);
        }
    }

    public Command presetAlgae(int level){
        return presetAlgae(level, ()->1);
    }

    public Command presetAlgaeIntake(int level, DoubleSupplier amount){
        switch (level) {
            case 1:
                return preset(Positions.AlgaeL, amount);
            case 2:
                return preset(Positions.AlgaeU, amount);
        
            default:
                return preset(Positions.Intake, amount);
        }
    }

    public Command presetAlgaeIntake(int level){
        return presetAlgae(level, ()->1);
    }

    public Command intake(){
        return Commands.either(Commands.none()
            ,Commands.parallel(toState(Positions.Intake)
            ,intake.setVelocity(IntakeSpeeds.IntakeCoral.getVelocity())
            ,jiggleRamp())
            .until(()->intake.hasCoral())
            .andThen(Commands.waitSeconds(INTAKE_TIME))
            .andThen(intake.stop())
            , ()->intake.hasAlgae());
    }

    public Command jiggleRamp(){
        return (ramp.reachGoal(RampPositions.Up.getPosition())
            .until(()->Math.abs(ramp.getTarget() - ramp.getPosition()) < .01)
            .andThen(ramp.reachGoal(RampPositions.Jiggle.getPosition())
                .until(()->Math.abs(ramp.getTarget() - ramp.getPosition()) < .01))).repeatedly();
    }

    public Command score(Positions positions, IntakeSpeeds speed){
        return toState(positions).andThen(intake.setVelocity(speed.getVelocity()).withTimeout(SHOOT_TIME));
    }

    public Command scoreCoral(int level){
        switch (level) {
            case 1:
                return score(Positions.L1, IntakeSpeeds.ShootCoral);
            case 2:
                return score(Positions.L2, IntakeSpeeds.ShootCoral);
            case 3:
                return score(Positions.L3, IntakeSpeeds.ShootCoral);
            case 4:
                return score(Positions.L4, IntakeSpeeds.ShootCoral);
        
            default:
                return preset(Positions.Intake);
        }
    }

    public Command scoreAlgae(int level){
        switch (level) {
            case 1:
                return score(Positions.AlgaeP, IntakeSpeeds.ShootAlgae);
            case 2:
                return score(Positions.AlgaeN, IntakeSpeeds.ShootAlgae);
        
            default:
                return preset(Positions.Intake);
        }
    }

    public Command intakeAlgae(Positions positions){
        return Commands.either(Commands.none()
            ,Commands.parallel(toState(positions)
            ,intake.setVelocity(IntakeSpeeds.IntakeAlgae.getVelocity()))
            .until(()->intake.hasAlgae())
            .andThen(intake.setVelocity(IntakeSpeeds.HoldAlgae.getVelocity()))
            , ()->intake.hasCoral());
    }

    public Command intakeAlgae(int level){
        switch (level) {
            case 1:
                return intakeAlgae(Positions.AlgaeL);
            case 2:
                return intakeAlgae(Positions.AlgaeU);
        
            default:
                return preset(Positions.Intake);
        }
    }

    public Command testPosition(DoubleSupplier elevatorMeters, DoubleSupplier armRotations, DoubleSupplier intakeSpeed){
        return elevator.reachGoal(elevatorMeters).alongWith(arm.reachGoal(armRotations)).alongWith(intake.setVelocity(intakeSpeed));
    }

    public void periodic(){
        if (elevatorNeedsReset && elevator.getPosition() < .2){
            elevatorNeedsReset = false;
        }else if (!elevatorNeedsReset && elevator.getPosition() > 2.4){
            elevatorNeedsReset = true;
        }
        resetPublisher.accept(elevatorNeedsReset);

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