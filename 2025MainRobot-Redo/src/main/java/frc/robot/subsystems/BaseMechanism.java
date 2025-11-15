package frc.robot.subsystems;

import java.util.Set;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

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
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Robot;
import frc.robot.constants.ArmConstants;
import frc.robot.constants.GameData;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.ramp.Ramp;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.util.PoseEX;
import frc.robot.util.mapleSimWorlds.MapleSim2025;

/**
 * every call pertaining to main scoring subsystems should be called through here
 */
public class BaseMechanism {

    protected final double MAX_ARM_ERROR = .05;
    protected final double MAX_ELEVATOR_ERROR = .05;
    protected final double MAX_RAMP_ERROR = .005;
    protected final double SHOOT_TIME = .4;
    protected final double INTAKE_TIME = .07;

    //The starting height of the end effector from ground
    public static final double ELEVATOR_END_EFFECTOR_OFFSETZ = .487;
    //The starting forward of the end effector from origin
    public static final double ELEVATOR_END_EFFECTOR_OFFSETX = .262;
    //The angle difference from tangent to the rotation of the arm
    public static final double END_EFFECTOR_SCORE_ANGLE = .05;
    //Where the end effector starts from the pivot, assuming straight up, then rotate back
    public static final double END_EFFECTOR_SCORE_OFFSET = -Units.degreesToRotations(32.8);//Rotations

    protected final Notifier notifier;

    public final Elevator elevator;
    public final Arm arm;
    public final Intake intake;
    public final Ramp ramp;
    public final SwerveDrive swerveDrive;

    public final Set<Subsystem> mainSubsystems;
    public final Set<Subsystem> presetSubsystems;

    protected final NetworkTable robot = NetworkTableInstance.getDefault().getTable("Robot");
    protected final NetworkTable armTable = robot.getSubTable("Arm");

    protected final StructSubscriber<Pose3d> armSubscriber = armTable.getStructTopic("ArmAngle",Pose3d.struct).subscribe(new Pose3d());

    protected final StructPublisher<Pose3d> coralPublisher = armTable.getStructTopic("CoralIfHad",Pose3d.struct).publish();

    protected final NetworkTable odom = robot.getSubTable("Odometry");
    protected StructSubscriber<Pose2d> poseSubscriber = odom
        .getStructTopic("RobotPose",Pose2d.struct).subscribe(new Pose2d());
    protected StructPublisher<Pose3d> targetPublisher = odom.getStructTopic("TargetPose",Pose3d.struct).publish();


    protected final double rampIntakePosition = -.142;
    protected final double rampRest = -.3;
    protected final double rampWiggleAmount = -.05;
    protected final double wigglePeriod = .05;

    public final double coralShootSpeed = 80;
    public final double algaeShootSpeed = -60;
    public final double coralIntakeSpeed = 60;
    public final double algaeIntakeSpeed = 60;
    public final double holdAlgaeSpeed = 20;

    protected final double armPressVolts = -3.5;
    
    public enum MainStates{
        Rest(0,0),
        Intake(0,0),
        L1(.2,0.2),
        L2(.4,0.15),
        L3(.8,0.15),
        L4(1.4,0.2),
        IntakeLowAlgae(.5,.5),
        IntakeHighAlgae(.8,.5),
        Processor(0,.25),
        Barge(0,0);

        public final double elevatorHeight;
        public final double armRotation;
        MainStates(double elevHeight, double armRotation){
            this.elevatorHeight = elevHeight;
            this.armRotation = armRotation;
        }
    }


    public BaseMechanism(Arm arm, Intake intake, Elevator elevator, Ramp ramp, SwerveDrive swerveDrive){
        this.elevator = elevator;
        this.arm = arm;
        this.intake = intake;
        this.ramp = ramp;
        this.swerveDrive = swerveDrive;

        mainSubsystems = Set.of(elevator, arm, intake, ramp, swerveDrive);
        presetSubsystems = Set.of(elevator, arm);

        elevator.setDefaultCommand(
            elevator.run(()->{
                boolean a = PoseEX.getDistanceFromPoseMeters(swerveDrive.getPose(),
                GameData.reefCenterPose()) > 1.5;
                if (a){
                    elevator.setPosition(0);
                }else{
                    elevator.setVolts(0);
                }
        }));

        arm.setDefaultCommand(arm.reachGoal(()->
            intake.hasAlgae() ? MainStates.Processor.armRotation : elevator.getPosition() > 2 ? MainStates.L4.armRotation : MainStates.Intake.armRotation
        ));
        intake.setDefaultCommand(intake.reachGoal(()->intake.hasAlgae() ? holdAlgaeSpeed : 0));

        if (Robot.isSimulation()){
            poseSubscriber = NetworkTableInstance.getDefault().getTable("RealData")
                .getStructTopic("RealPose", Pose2d.struct).subscribe(new Pose2d());
            MapleSim2025.addIntakeSimulation("CoralIntake", "Coral", .5,.4, new Translation2d(-.1,0));
            // MapleSimWorld.addShooterSimulation(()->
            //     new Transform3d(0.2 + (arm.getPosition() > 1.0/7.0 ? .29 : 0),0, elevator.getPositionMeters() + ELEVATOR_END_DEFFECTOR_OFFSET,new Rotation3d(0, (arm.getPosition() > 1.0/7.0 ? -Math.PI/2 : -Units.rotationsToRadians(arm.getPosition())),0))
            //     , ()->2
            //     , "Coral"
            //     , "CoralIntake");
            MapleSim2025.addShooterSimulation(()->{
                    Pose3d armPose = armSubscriber.get(new Pose3d());
                    double yOffset = ArmConstants.ARM_LENGTH_METERS * Math.cos(Units.rotationsToRadians(END_EFFECTOR_SCORE_OFFSET) + armPose.getRotation().getY());
                    double xOffset = ArmConstants.ARM_LENGTH_METERS * Math.sin(Units.rotationsToRadians(END_EFFECTOR_SCORE_OFFSET) + armPose.getRotation().getY());
                        return new Transform3d(armPose.getX() + xOffset-.05,0, armPose.getZ() + yOffset
                        ,new Rotation3d(0, armPose.getRotation().getY() + Units.rotationsToRadians(END_EFFECTOR_SCORE_ANGLE)+Math.PI,armSubscriber.get().getZ() > 1.5 ? 0 : Math.PI));
                }
                // Pose3d armPose = armSubscriber.get(new Pose3d());
                // double yOffset = ArmConstants.ARM_LENGTH_METERS * Math.cos(Units.rotationsToRadians(ArmConstants.ARM_END_DEFFECTOR_SCORE_OFFSET) + armPose.getRotation().getY());
                // double xOffset = ArmConstants.ARM_LENGTH_METERS * Math.sin(Units.rotationsToRadians(ArmConstants.ARM_END_DEFFECTOR_SCORE_OFFSET) + armPose.getRotation().getY());
                // return new Transform3d(armPose.getX() + xOffset,0, armPose.getZ() + yOffset
                //     ,new Rotation3d(0, armPose.getRotation().getY() + Units.rotationsToRadians(ArmConstants.ARM_END_DEFFECTOR_SCORE_ANGLE),0));
                // }
                
                , ()->4
                , "Coral"
                , "CoralIntake");
            MapleSim2025.addIntakeRequirements("CoralIntake", ()->intake.getVelocity() > 50);
            MapleSim2025.addIntakeRequirements("CoralIntake", ()->Math.abs(arm.getPosition() - MainStates.Intake.armRotation) < .1);
            MapleSim2025.addIntakeRequirements("CoralIntake", ()->Math.abs(elevator.getPosition() - MainStates.Intake.elevatorHeight) < .1);
            MapleSim2025.hasPiece("CoralIntake",(has)->intake.getCoralDigitalInputIO().setValue(has));
            MapleSim2025.addShootRequirements("CoralIntake", ()->intake.getVelocity() > 70);

            MapleSim2025.addIntakeSimulation("AlgaeIntake","Algae", .5,.4,new Translation2d(.3,0));
            MapleSim2025.addIntakeRequirements("AlgaeIntake", ()->intake.getVelocity() < -20);
            MapleSim2025.hasPiece("AlgaeIntake",(has)->intake.getAlgaeDigitalInputIO().setValue(has));

            MapleSim2025.addShooterSimulation(()->
                new Transform3d(ELEVATOR_END_EFFECTOR_OFFSETX,0, elevator.getPosition() + ELEVATOR_END_EFFECTOR_OFFSETZ,new Rotation3d(0, -Units.rotationsToRadians(arm.getPosition()),0))
                , ()->2
                , "Algae"
                , "AlgaeIntake");
            MapleSim2025.addShootRequirements("AlgaeIntake", ()->intake.getVelocity() > 10);
        }

        notifier = new Notifier(this :: periodic);
        notifier.setName("Scoring Periodic");
        notifier.startPeriodic(.02);
        Runtime.getRuntime().addShutdownHook(new Thread(notifier::close));
    }

    // public Command configure(){

    // }


    ////BASE reachState methods
    
    /**
     * set the mechanism state
     * @param armPosition
     * @param elevatorPosition
     * @param intakeSpeed
     * @param rampPosition
     * @return
     */
    public Command reachState(DoubleSupplier armPosition, DoubleSupplier elevatorPosition){
        return Commands.parallel(arm.reachGoal(armPosition), elevator.reachGoal(elevatorPosition));
    }

    public Command reachState(double armPosition, double elevatorPosition){
        return reachState(()->armPosition, ()->elevatorPosition);
    }

    /**
     * set the robot state
     * @param armPosition
     * @param elevatorPosition
     * @param intakeSpeed
     * @param rampPosition
     * @param robotPosition
     * @param maxSpeed
     * @param maxTurnRads
     * @return
     */
    public Command reachState(DoubleSupplier armPosition, DoubleSupplier elevatorPosition, Supplier<Pose2d> robotPosition,
        double maxSpeed, double maxTurnRads){

        return Commands.parallel(reachState(armPosition, elevatorPosition)
            , swerveDrive.towardPose(robotPosition, maxSpeed, maxTurnRads));
    }

    public Command reachState(double armPosition, double elevatorPosition, Pose2d robotPose, double maxSpeed, double maxTurnRads){
        return reachState(()->armPosition, ()->elevatorPosition, ()->robotPose, maxSpeed, maxTurnRads);
    }

    public boolean inState(double armRotation, double armMaxError, double elevatorHeight, double elevatorMaxError){
        if (Math.abs(arm.getPosition() - armRotation) > armMaxError){
            return false;
        }
        if (Math.abs(elevator.getPosition() - elevatorHeight) > elevatorMaxError){
            return false;
        }
        return true;
    }

    
    ////DEFAULT SCORING AND INTAKE POSITIONS
    
    public boolean inState(MainStates state, double armMaxError, double elevatorMaxError){
        return inState(state.armRotation, armMaxError, state.elevatorHeight, elevatorMaxError);
    }

    public Command setState(MainStates state){
        return reachState(state.armRotation, state.elevatorHeight);
    }

    public Command setCoralState(int level){
        int l = MathUtil.clamp(level, 1, 4);
        if (l == 4){
            return setState(MainStates.L4);
        }else if (l == 3){
            return setState(MainStates.L3);
        }else if (l == 2){
            return setState(MainStates.L2);
        }else{
            return setState(MainStates.L1);
        }
    }

    public Command setAlgaeIntakeState(int level){
        int l = MathUtil.clamp(level, 1, 2);
        if (l == 2){
            return setState(MainStates.IntakeHighAlgae);
        }else{
            return setState(MainStates.IntakeLowAlgae);
        }
    }

    public Command setAlgaeScoreState(int level){
        int l = MathUtil.clamp(level, 1, 2);
        if (l == 2){
            return setState(MainStates.Barge);
        }else{
            return setState(MainStates.Processor);
        }
    }

    public Command intake(){
        return Commands.parallel(intake.reachGoal(coralIntakeSpeed), rampWiggle()
                , setState(MainStates.Intake).until(()->inState(MainStates.Intake, MAX_ARM_ERROR, MAX_ELEVATOR_ERROR) && Math.abs(ramp.getPosition()) < MAX_RAMP_ERROR)
                    .andThen(arm.setVoltage(armPressVolts).alongWith(elevator.reachGoal(MainStates.Intake.elevatorHeight))))
            .until(()->intake.hasCoral())
            .andThen(Commands.waitSeconds(INTAKE_TIME))
            .andThen(intake.stop());
    }

    public Command intakeAlgae(int level){
        return Commands.parallel(Commands.either(setState(MainStates.IntakeLowAlgae), setState(MainStates.IntakeHighAlgae),()-> MathUtil.clamp(level, 1, 2) == 1)
            ,intake.reachGoal(algaeIntakeSpeed)).until(()->intake.hasAlgae()).andThen(intake.reachGoalOnce(holdAlgaeSpeed));
    }

    public Command rampWiggle(){
        //TODO, the repeatedly might cause errors, if any check here
        return (ramp.reachGoal(rampIntakePosition).withTimeout(wigglePeriod/2).andThen(ramp.reachGoal(rampIntakePosition+rampWiggleAmount).withTimeout(wigglePeriod/2))).repeatedly();
    }

    public void periodic(){
        if (intake.hasCoral()){
            Pose3d armPose = armSubscriber.get(new Pose3d());
                double yOffset = ArmConstants.ARM_LENGTH_METERS * Math.cos(Units.rotationsToRadians(END_EFFECTOR_SCORE_OFFSET) + armPose.getRotation().getY());
                double xOffset = ArmConstants.ARM_LENGTH_METERS * Math.sin(Units.rotationsToRadians(END_EFFECTOR_SCORE_OFFSET) + armPose.getRotation().getY());
            Transform3d t = new Transform3d(armPose.getX() + xOffset,0, armPose.getZ() + yOffset
            ,new Rotation3d(0, armPose.getRotation().getY() + Units.rotationsToRadians(END_EFFECTOR_SCORE_ANGLE),0));
            coralPublisher.accept(new Pose3d(poseSubscriber.get()).plus(t));
        }else{
            coralPublisher.accept(new Pose3d());
        }
    }
}