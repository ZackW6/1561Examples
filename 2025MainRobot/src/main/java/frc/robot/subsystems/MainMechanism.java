package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amp;

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
import edu.wpi.first.units.measure.Velocity;
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

// /**
//  * every call pertaining to main scoring subsystems should be called through here
//  */
// public class MainMechanism {

//     private final Notifier notifier;

//     private final double MAX_ARM_ERROR = .05;
//     private final double MAX_ELEVATOR_ERROR = .05;
//     private final double MAX_RAMP_ERROR = .05;
//     private final double MAX_SIGNAL_TIME = .03;
//     private final double SHOOT_TIME = .3;
//     public static final double ELEVATOR_END_DEFFECTOR_OFFSET = .6;

//     public final Arm arm;

//     public final Intake intake;

//     public final Elevator elevator;

//     public final Ramp ramp;

//     private final NetworkTable robot = NetworkTableInstance.getDefault().getTable("Robot");
//     private final NetworkTable armTable = robot.getSubTable("Arm");

//     private final StructSubscriber<Pose3d> armSubscriber = armTable.getStructTopic("ArmAngle",Pose3d.struct).subscribe(new Pose3d());

//     private final StructPublisher<Pose3d> coralPublisher = armTable.getStructTopic("CoralIfHad",Pose3d.struct).publish();

//     private final NetworkTable odom = robot.getSubTable("Odometry");
//     private StructSubscriber<Pose2d> poseSubscriber = odom
//         .getStructTopic("RobotPose",Pose2d.struct).subscribe(new Pose2d());

//     private final double rampDownPosition = 0;
//     private final double rampJigglePosition = -.04;

//     public static enum Positions{
//         CoralReset(.19444,0),
//         Intake(0,0),
//         L1(0,.1),
//         L2(.09722,.5),
//         L3(.09722,1),
//         L4(.19444,1.5),
//         AlgaeReset(.5,0),
//         AlgaeU(.125,.6),
//         AlgaeL(.125,.3),
//         AlgaeN(-.05555,2),
//         AlgaeP(.05555,0);

//         private double armRotations;
//         private double elevatorMeters;
//         Positions(double armRotations, double elevatorMeters){
//             this.armRotations = armRotations;
//             this.elevatorMeters = elevatorMeters;
//         }

//         public double armRotations(){
//             return armRotations;
//         }

//         public double elevatorMeters(){
//             return elevatorMeters;
//         }
//     }

//     public static enum IntakeSpeeds{
//         Idle(0),
//         IntakeCoral(7.5),
//         IntakeAlgae(-30),
//         HoldAlgae(-10),
//         Shoot(30);

//         private double velocity;
//         IntakeSpeeds(double velocityRPS){
//             this.velocity = velocityRPS;
//         }

//         public double value(){
//             return velocity;
//         }
//     }

//     public MainMechanism(Arm arm, Intake intake, Elevator elevator, Ramp ramp){
//         notifier = new Notifier(this::periodic);
//         notifier.startPeriodic(0.02);

//         this.arm = arm;
//         this.intake = intake;
//         this.elevator = elevator;
//         this.ramp = ramp;
        
//         arm.setDefaultCommand(arm.reachGoal(()->{
//             if (elevator.getPosition() < .2){
//                 return Positions.Intake.armRotations();
//             }
//             return Positions.L4.armRotations();
//         }));
//         intake.setDefaultCommand(intake.setVelocity(()->intake.hasAlgae() ? IntakeSpeeds.HoldAlgae.value() : 0));
//         elevator.setDefaultCommand(elevator.reachGoal(Positions.Intake.elevatorMeters()));
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
//                 Pose3d armPose = armSubscriber.get(new Pose3d());
//                 double yOffset = ArmConstants.ARM_LENGTH_METERS * Math.cos(Units.rotationsToRadians(ArmConstants.ARM_END_DEFFECTOR_SCORE_OFFSET) + armPose.getRotation().getY());
//                 double xOffset = ArmConstants.ARM_LENGTH_METERS * Math.sin(Units.rotationsToRadians(ArmConstants.ARM_END_DEFFECTOR_SCORE_OFFSET) + armPose.getRotation().getY());
//                 return new Transform3d(armPose.getX() + xOffset,0, armPose.getZ() + yOffset
//                     ,new Rotation3d(0, -armPose.getRotation().getY() - Units.rotationsToRadians(ArmConstants.ARM_END_DEFFECTOR_SCORE_ANGLE),0));
//                 }
                
//                 , ()->2
//                 , "Coral"
//                 , "CoralIntake");
//             MapleSimWorld.addIntakeRequirements("CoralIntake", ()->intake.getVelocity() > 5);
//             MapleSimWorld.addIntakeRequirements("CoralIntake", ()->Math.abs(arm.getPosition() - Positions.Intake.armRotations()) < .1);
//             MapleSimWorld.addIntakeRequirements("CoralIntake", ()->Math.abs(elevator.getPosition() - Positions.Intake.elevatorMeters) < .1);
//             MapleSimWorld.hasPiece("CoralIntake",(has)->intake.getCoralDigitalInputIO().setValue(has));
//             MapleSimWorld.addShootRequirements("CoralIntake", ()->intake.getVelocity() > 10);

//             // MapleSimWorld.addIntakeSimulation("AlgaeIntake","Algae", .5,.4,new Translation2d(.3,0));
//             // MapleSimWorld.addIntakeRequirements("AlgaeIntake", ()->intake.getVelocity() < -20);
//             // MapleSimWorld.hasPiece("AlgaeIntake",(has)->intake.getAlgaeDigitalInputIO().setValue(has));

//             // MapleSimWorld.addShooterSimulation(()->
//             //     new Transform3d(0.2,0, elevator.getPositionMeters() + ELEVATOR_END_DEFFECTOR_OFFSET,new Rotation3d(0, -Units.rotationsToRadians(arm.getPosition()),0))
//             //     , ()->2
//             //     , "Algae"
//             //     , "AlgaeIntake");
//             // MapleSimWorld.addShootRequirements("AlgaeIntake", ()->intake.getVelocity() > 10);


//         }
//         Runtime.getRuntime().addShutdownHook(new Thread(notifier::close));
//     }

//     /**
//      * keep in mind, toState ends automatically!!!
//      * @param position
//      * @return
//      */
//     private Command toState(Positions position){
//         return toSafeState(position, Positions.L4, ()->1);//Commands.either(Commands.none(), toStateCoral(position),()-> intake.hasAlgae());
//     }

//     /**
//      * keep in mind, toState ends automatically!!!
//      * @param position
//      * @return
//      */
//     private Command toStateCoral(Positions position){
//         return arm.reachGoal(Positions.L4.armRotations()) // Goes to the other side of the elevator top bar
//             .until(()->Math.abs(arm.getTarget()-arm.getPosition()) < MAX_ARM_ERROR)
//             .andThen(elevator.reachGoal(position.elevatorMeters()))
//             .until(()->Math.abs(position.elevatorMeters()-elevator.getPosition()) < MAX_ELEVATOR_ERROR)
//             .andThen(arm.reachGoalOnce(position.armRotations()));
//     }

//     private Command toSafeState(Positions positions, Positions safe, DoubleSupplier amount){
//         return arm.reachGoal(safe.armRotations()).until(()->Math.abs(arm.getTarget() - arm.getPosition()) < MAX_ARM_ERROR)
//             .andThen(elevator.reachGoal(()->positions.elevatorMeters()*MathUtil.clamp(amount.getAsDouble(),0,1))).until(()->Math.abs(positions.elevatorMeters() - elevator.getPosition()) < MAX_ELEVATOR_ERROR)
//             .andThen(arm.reachGoalOnce(positions.armRotations()));
//     }

//     public Command idle(){
//         return Commands.either(Commands.none(),toState(Positions.Intake)
//             .alongWith(intake.setVelocity(IntakeSpeeds.Idle.value())),()->intake.hasAlgae());
//     }

//     /**
//      * ends after you get a piece
//      * @return
//      */
//     public Command intake(){
//         return Commands.either(Commands.none()
//         , Commands.parallel(
//             toState(Positions.Intake)
//             ,intake.setVelocity(IntakeSpeeds.IntakeCoral.value())
//             ,jiggleRamp()).until(()->intake.hasCoral())
//             .andThen(intake.stop())
//             ,()->intake.hasAlgae());
//     }

//     public Command jiggleRamp(){
//         return (ramp.reachGoal(rampDownPosition)
//             .until(()->Math.abs(ramp.getTarget() - ramp.getPosition()) < MAX_RAMP_ERROR)
//             .andThen(ramp.reachGoal(rampJigglePosition)
//                 .until(()->Math.abs(ramp.getTarget() - ramp.getPosition()) < MAX_RAMP_ERROR))).repeatedly();
//     }

//     public Command scoreL1(){
//         return score(Positions.L1);
//     }
//     public Command presetL1(){
//         return preset(Positions.L1);
//     }

//     public Command scoreL2(){
//         return score(Positions.L2);
//     }
//     public Command presetL2(){
//         return preset(Positions.L2);
//     }

//     public Command scoreL3(){
//         return score(Positions.L3);
//     }
//     public Command presetL3(){
//         return preset(Positions.L3);
//     }

//     public Command scoreL4(){
//         return score(Positions.L4);
//     }
//     public Command presetL4(){
//         return preset(Positions.L4);
//     }

//     public Command score(Positions position){
//         return toState(position)
//             .andThen(Commands.waitUntil(()->Math.abs(position.armRotations()-arm.getPosition()) < MAX_ARM_ERROR 
//                 && Math.abs(elevator.getTarget()-elevator.getPosition()) < MAX_ELEVATOR_ERROR))
//             .andThen(intake.setVelocity(IntakeSpeeds.Shoot.value()).withTimeout(SHOOT_TIME));
//     }

//     /**
//      * does not end automatically
//      * @param position
//      * @return
//      */
//     public Command preset(Positions position){
//         return preset(position, ()->1);
//     }

//     /**
//      * does not end automatically, this slowly rises with 0 being 0% and 100% 
//      * @param position
//      * @return
//      */
//     public Command preset(Positions position, DoubleSupplier amount){
//         return arm.reachGoal(Positions.L4.armRotations())
//             .until(()->Math.abs(arm.getPosition()-Positions.L4.armRotations()) < MAX_ARM_ERROR)
//             .andThen(elevator.reachGoal(()->position.elevatorMeters()*MathUtil.clamp(amount.getAsDouble(), 0, 1)))
//             .until(()->Math.abs(position.elevatorMeters()-elevator.getPosition()) < MAX_ELEVATOR_ERROR)
//             .andThen(arm.reachGoal(position.armRotations()));
//     }

//     public Command presetCoral(int level, DoubleSupplier amount){
//         if (level == 4){
//             return preset(Positions.L4, amount);
//         }
//         if (level == 3){
//             return preset(Positions.L3, amount);
//         }
//         if (level == 2){
//             return preset(Positions.L2, amount);
//         }
//         return preset(Positions.L1, amount);
//     }

//     public Command scoreCoral(int level){
//         switch (level) {
//             case 4:
//                 return scoreL4();
//             case 3:
//                 return scoreL3();
//             case 2:
//                 return scoreL2();        
//             default:
//                 return scoreL1();
//         }
//     }

//     public Command presetCoral(int level){
//         switch (level) {
//             case 4:
//                 return presetL4();
//             case 3:
//                 return presetL3();
//             case 2:
//                 return presetL2();        
//             default:
//                 return presetL1();
//         }
//     }

//     public Command presetAlgaeUpper(){
//         return preset(Positions.AlgaeU);
//     }

//     /**
//      * ends after you get a piece
//      * @return
//      */
//     public Command intakeAlgae(Positions level){
//         return Commands.either(Commands.none(),Commands.parallel(toState(level),
//             intake.setVelocity(IntakeSpeeds.IntakeAlgae.value())).until(()->intake.hasAlgae()),()->intake.hasAlgae() || intake.hasCoral());
//     }

//     /**
//      * ends after you get a piece
//      * @return
//      */
//     public Command intakeAlgae(int level){
//         level = MathUtil.clamp(level, 1, 2);
//         if (level == 1){
//             return intakeAlgae(Positions.AlgaeL);
//         }else{
//             return intakeAlgae(Positions.AlgaeU);
//         }
//     }

//     /**
//      * ends after you get a piece
//      * @return
//      */
//     public Command scoreAlgae(int level){
//         level = MathUtil.clamp(level, 1, 2);
//         if (level == 1){
//             return scoreAlgaeProcessor();
//         }else{
//             return scoreAlgaeNet();
//         }
//     }

//     /**
//      * ends after you get a piece
//      * @return
//      */
//     public Command presetAlgae(int level){
//         level = MathUtil.clamp(level, 1, 2);
//         if (level == 1){
//             return presetAlgaeProcessor();
//         }else{
//             return presetAlgaeNet();
//         }
//     }

//     /**
//      * ends after you get a piece
//      * @return
//      */
//     public Command intakeAlgaeUpper(){
//         return intakeAlgae(Positions.AlgaeU);
//     }

//     public Command presetAlgaeLower(){
//         return preset(Positions.AlgaeL);
//     }
//     /**
//      * ends after you get a piece
//      * @return
//      */
//     public Command intakeAlgaeLower(){
//         return intakeAlgae(Positions.AlgaeL);
//     }

//     public Command presetAlgaeNet(){
//         return preset(Positions.AlgaeN);
//     }
//     public Command scoreAlgaeNet(){
//         return score(Positions.AlgaeN);
//     }

//     public Command presetAlgaeProcessor(){
//         return preset(Positions.AlgaeP);
//     }
//     public Command scoreAlgaeProcessor(){
//         return score(Positions.AlgaeP);
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

    private final double MAX_ARM_ERROR = .05;
    private final double MAX_ELEVATOR_ERROR = .05;
    private final double MAX_RAMP_ERROR = .05;
    private final double MAX_SIGNAL_TIME = .03;
    private final double SHOOT_TIME = .4;
    public static final double ELEVATOR_END_DEFFECTOR_OFFSET = .6;

    private final Notifier notifier;

    public final Elevator elevator;
    public final Arm arm;
    public final Intake intake;
    public final Ramp ramp;

    private final NetworkTable robot = NetworkTableInstance.getDefault().getTable("Robot");
    private final NetworkTable armTable = robot.getSubTable("Arm");

    private final StructSubscriber<Pose3d> armSubscriber = armTable.getStructTopic("ArmAngle",Pose3d.struct).subscribe(new Pose3d());

    private final StructPublisher<Pose3d> coralPublisher = armTable.getStructTopic("CoralIfHad",Pose3d.struct).publish();

    private final NetworkTable odom = robot.getSubTable("Odometry");
    private StructSubscriber<Pose2d> poseSubscriber = odom
        .getStructTopic("RobotPose",Pose2d.struct).subscribe(new Pose2d());

    public static enum Positions{
        
        CoralReset(.1944,0, false),
        Idle(0,0, false),
        Intake(0,0, false),
        L1(0,.1, false),
        L2(.09722,.5, false),
        L3(.09722,1, false),
        L4(.1944,1.6, false),
        AlgaeReset(.5,0),
        AlgaeU(.5,.6),
        AlgaeL(.5,.3),
        AlgaeN(-.0555,2),
        AlgaeP(.0555,0);

        public static final double safeStartArmRotations = 0;

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
        IntakeCoral(7.5),
        ShootCoral(30),
        IntakeAlgae(-15),
        HoldAlgae(-10),
        ShootAlgae(40);

        private final double velocity;
        IntakeSpeeds(double velocity){
            this.velocity = velocity;
        }

        public double getVelocity(){
            return velocity;
        }
    }

    public static enum RampPositions{
        Up(0),
        Down(.25),
        Jiggle(.01);

        private final double rotation;
        RampPositions(double rotation){
            this.rotation = rotation;
        }

        public double getPosition(){
            return rotation;
        }
    }

    public MainMechanism(Arm arm, Intake intake, Elevator elevator, Ramp ramp){
        this.elevator = elevator;
        this.arm = arm;
        this.intake = intake;
        this.ramp = ramp;

        elevator.setDefaultCommand(elevator.reachGoal(()->Math.abs(arm.getPosition() - arm.getTarget()) < MAX_ARM_ERROR ? Positions.Intake.elevatorMeters() : elevator.getPosition()));
        arm.setDefaultCommand(arm.reachGoal(()->
            elevator.getPosition() > .2 ? intake.hasAlgae() ? Positions.AlgaeReset.armRotations() : Positions.CoralReset.armRotations() : intake.hasAlgae() ? Positions.AlgaeReset.armRotations() : Positions.Intake.armRotations()
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
                return new Transform3d(armPose.getX() + xOffset,0, armPose.getZ() + yOffset
                    ,new Rotation3d(0, -armPose.getRotation().getY() - Units.rotationsToRadians(ArmConstants.ARM_END_DEFFECTOR_SCORE_ANGLE),0));
                }
                
                , ()->2
                , "Coral"
                , "CoralIntake");
            MapleSimWorld.addIntakeRequirements("CoralIntake", ()->intake.getVelocity() > 5);
            MapleSimWorld.addIntakeRequirements("CoralIntake", ()->Math.abs(arm.getPosition() - Positions.Intake.armRotations()) < .1);
            MapleSimWorld.addIntakeRequirements("CoralIntake", ()->Math.abs(elevator.getPosition() - Positions.Intake.elevatorMeters) < .1);
            MapleSimWorld.hasPiece("CoralIntake",(has)->intake.getCoralDigitalInputIO().setValue(has));
            MapleSimWorld.addShootRequirements("CoralIntake", ()->intake.getVelocity() > 10);

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

    /**
     * ends when it has made each subsytem act in the wanted manner, purely goes to a position without any hits. (A preset)
     * @param positions
     * @return
     */
    public Command toState(Positions positions, DoubleSupplier amount, boolean ending){
        return Commands.either(Commands.either(toSafeState(positions, Positions.AlgaeReset, amount, ending)
                , Commands.none()
                , ()->positions.algaeSafe())
            , toSafeState(positions, Positions.CoralReset, amount, ending)
            , ()->intake.hasAlgae());
    }

    /**
     * ends when it has made each subsytem act in the wanted manner, purely goes to a position without any hits. (A preset)
     * @param positions
     * @return
     */
    public Command toState(Positions positions){
        return toState(positions, ()->1, true);
    }

    private Command toSafeState(Positions positions, Positions safe, DoubleSupplier amount, boolean ending){
        return arm.reachGoal(safe.armRotations()).until(()->Math.abs(arm.getTarget() - arm.getPosition()) < MAX_ARM_ERROR)
            .andThen(elevator.reachGoal(()->positions.elevatorMeters()*MathUtil.clamp(amount.getAsDouble(),0,1))).until(()->Math.abs(positions.elevatorMeters() - elevator.getPosition()) < MAX_ELEVATOR_ERROR)
            .andThen(Commands.either(
                arm.reachGoalOnce(positions.armRotations())
                , elevator.reachGoal(()->positions.elevatorMeters()*MathUtil.clamp(amount.getAsDouble(),0,1))
                    .alongWith(arm.reachGoal(()->Math.abs(positions.elevatorMeters() - elevator.getPosition()) < MAX_ELEVATOR_ERROR ? positions.armRotations() : safe.armRotations()))
                , ()->ending));
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
            ,jiggleRamp()).until(()->intake.hasCoral())
            .andThen(intake.stop())
            , ()->intake.hasAlgae());
    }

    public Command jiggleRamp(){
        return (ramp.reachGoal(RampPositions.Up.getPosition())
            .until(()->Math.abs(ramp.getTarget() - ramp.getPosition()) < MAX_RAMP_ERROR)
            .andThen(ramp.reachGoal(RampPositions.Up.getPosition())
                .until(()->Math.abs(ramp.getTarget() - ramp.getPosition()) < MAX_RAMP_ERROR))).repeatedly();
    }

    public Command score(Positions positions, IntakeSpeeds speed){
        return toState(positions).andThen(intake.setVelocity(speed.getVelocity())).withTimeout(SHOOT_TIME);
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