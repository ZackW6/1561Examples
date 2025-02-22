package frc.robot.commands;

import java.util.Optional;
import java.util.Set;
import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ApplyFieldSpeeds;
import com.ctre.phoenix6.swerve.SwerveRequest.ApplyRobotSpeeds;
import com.pathplanner.lib.auto.CommandUtil;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.commands.PathOnTheFly.PathConfig;
import frc.robot.constants.GameData;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.MainMechanism;
import frc.robot.subsystems.MainMechanism.Positions;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.subsystems.swerve.realSwerve.CommandSwerveDrivetrain;
import frc.robot.subsystems.swerve.swerveHelpers.ShareDrive;
import frc.robot.subsystems.swerve.swerveHelpers.Vector2;
import frc.robot.util.PoseEX;

public class FactoryCommands {

    public static final double positionalToleranceMeters = .05;
    public static final double rotationalToleranceRotations = .05;

    public final SwerveDrive drivetrain;

    public final MainMechanism scoringMechanism;

    public final Set<Subsystem> scoringSubsytems;

    private static FactoryCommands instance;
    
    private final ApplyFieldSpeeds fieldSpeedRequest = new ApplyFieldSpeeds();

    private final ApplyRobotSpeeds robotSpeedRequest = new ApplyRobotSpeeds();

    private final ShareDrive shareRequest;

    public FactoryCommands(SwerveDrive drivetrain, MainMechanism scoringMechanism){
        if (instance == null){
            instance = this;
        }
        this.drivetrain = drivetrain;
        this.scoringMechanism = scoringMechanism;
        this.shareRequest = new ShareDrive(()->drivetrain.getPose().getRotation(),()->drivetrain.getDriveIO().getYawOffset())
            .withMaxLinearVelocity(TunerConstants.kSpeedAt12VoltsMps)
            .withMaxRotationalVelocity(TunerConstants.MAX_ANGULAR_RATE);

        scoringSubsytems = Set.of(drivetrain, scoringMechanism.arm, scoringMechanism.elevator, scoringMechanism.intake);
    }

    public static Optional<FactoryCommands> getInstance(){
        if (instance == null){
            return Optional.empty();
        }
        return Optional.of(instance);
    }

    private final PIDController speedsPID = new PIDController(5, 0, 0);
    private final PIDController rotationPID = new PIDController(10, 0, 0);

    /**
     * @param pose
     * @param strengthCompWithNormalDrive
     * @param minDist
     * @param maxAngle
     * @return
     */
    public Command towardPose(Pose2d pose, double speedCap, double radianPSCap, double minDist){
        return Commands.run(()->{

            Vector2 vector = new Vector2(pose.getX()-drivetrain.getPose().getX(), pose.getY()-drivetrain.getPose().getY());
            if (vector.getMagnitude() > minDist){
                drivetrain.setControl(fieldSpeedRequest.withSpeeds(new ChassisSpeeds()));
            }

            Vector2 finalSpeeds = new Vector2(-speedsPID.calculate(vector.x,0),-speedsPID.calculate(vector.y,0));
            if (finalSpeeds.getMagnitude() > speedCap){
                finalSpeeds = finalSpeeds.normalize().multiply(speedCap);
            }

            double finalRotation = Math.min(Math.max(rotationPID.calculate(distToCorrectedPoint(pose.getRotation().getRadians(), drivetrain.getPose().getRotation().getRadians()),0),-radianPSCap),radianPSCap);

            drivetrain.setControl(fieldSpeedRequest.withSpeeds(new ChassisSpeeds(finalSpeeds.x,
                finalSpeeds.y,
                finalRotation)));

        },drivetrain);
    }

    /**
     * @param translation
     * @param strengthCompWithNormalDrive
     * @param minDist
     * @param maxAngle
     * @return
     */
    public Command passiveTowardTranslation(Translation2d translation, double speedCap, double radianPSCap, double minDist){
        return Commands.run(()->{

            Vector2 vector = new Vector2(translation.getX()-drivetrain.getPose().getX(), translation.getY()-drivetrain.getPose().getY());
            if (vector.getMagnitude() > minDist){
                drivetrain.setControl(fieldSpeedRequest.withSpeeds(new ChassisSpeeds()));
                return;
            }

            Vector2 finalSpeeds = new Vector2(-speedsPID.calculate(vector.x,0),-speedsPID.calculate(vector.y,0));
            if (finalSpeeds.getMagnitude() > speedCap){
                finalSpeeds = finalSpeeds.normalize().multiply(speedCap);
            }


            drivetrain.setControl(fieldSpeedRequest.withSpeeds(new ChassisSpeeds(finalSpeeds.x, finalSpeeds.y, 0)));

        },drivetrain);
    }

    private double distToCorrectedPoint(double pos, double goal){
        if (Math.abs(goal-pos) < Math.PI){
            return goal-pos;
        }
        if (pos < 0){
            return goal-Math.PI*2-pos;
        }else{
            return goal+Math.PI*2-pos;
        }
    }

    /**
     * does not end automatically, must be ended
     * @return
     */
    public Command toPose(Pose2d pose, double straightDist){
        return PathOnTheFly.AutoToPoint.getToPoint(pose, new PathConfig(4,5,
            Rotation2d.fromRadians(3*Math.PI),Rotation2d.fromDegrees(720),0,0))
            .until(()->drivetrain.getPose().minus(pose).getTranslation().getNorm() < straightDist)
            .andThen(towardPose(pose, 5, 3*Math.PI, 5));
    }

    public Command autoToCoral(int place){
        int clampedNum = Math.max(Math.min(place,12),1);
        return Commands.defer(()->toPose(GameData.coralPose(clampedNum, DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red)
        ,1.2),Set.of());
    }

    public Command autoToAlgae(int place){
        int clampedNum = Math.max(Math.min(place,6),1);
        return Commands.defer(()->toPose(GameData.algaePose(clampedNum, DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red)
        ,1.2),Set.of());
    }

    public Command autoToFeeder(int place, double rightOffset){
        int clampedNum = Math.max(Math.min(place,2),1);
        return Commands.defer(()->toPose(GameData.feederPose(clampedNum, DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red)
        .plus(new Transform2d(0,rightOffset, new Rotation2d()))
        ,3),Set.of());
    }

    public Command autoToFeeder(int place){
        return autoToFeeder(place,0);
    }

    public Command autoToProcessor(){
        return Commands.defer(()->toPose(GameData.processorPose(DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red)
        ,3),Set.of());
    }

    public Command autoToNet(){
        return Commands.defer(()->toPose(
            PoseEX.closestTo(drivetrain.getPose(),
            GameData.netPose(1,DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red),
            GameData.netPose(2,DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red))
        ,2),Set.of());
    }

    public Command autoScoreCoral(int place, int level){
        return Commands.race(autoToCoral(place)
        ,Commands.race(Commands.waitUntil(()->{
            Pose2d drivetrainPose = drivetrain.getPose();
            Pose2d coralPose = GameData.coralPose(place, DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red);
            Transform2d comparingTransform = coralPose.minus(drivetrainPose);

            return (comparingTransform.getTranslation().getNorm() < positionalToleranceMeters) 
                && (coralPose.getRotation().getRotations() - drivetrainPose.getRotation().getRotations() < rotationalToleranceRotations)
                && drivetrain.getDriveIO().getSpeeds().vyMetersPerSecond < .1;
        })
        ,scoringMechanism.preset(level,()->{
            Pose2d drivetrainPose = drivetrain.getPose();
            Pose2d coralPose = GameData.coralPose(place, DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red);
            Transform2d comparingTransform = coralPose.minus(drivetrainPose);

            return 1/Math.min(comparingTransform.getTranslation().getNorm(),5);
        })).andThen(scoringMechanism.score(level)));
    }

    public Command autoScoreNet(){
        return Commands.race(autoToNet()
        ,Commands.race(Commands.waitUntil(()->{
            Pose2d drivetrainPose = drivetrain.getPose();
            Pose2d algaePose = PoseEX.closestTo(drivetrain.getPose(),
                GameData.netPose(1,DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red),
                GameData.netPose(2,DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red));
            Transform2d comparingTransform = algaePose.minus(drivetrainPose);

            return (comparingTransform.getTranslation().getNorm() < positionalToleranceMeters) 
                && (algaePose.getRotation().getRotations() - drivetrainPose.getRotation().getRotations() < rotationalToleranceRotations);
        })
        ,scoringMechanism.preset(Positions.AlgaeN,()->{
            Pose2d drivetrainPose = drivetrain.getPose();
            Pose2d algaePose = PoseEX.closestTo(drivetrain.getPose(),
                GameData.netPose(1,DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red),
                GameData.netPose(2,DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red));
            Transform2d comparingTransform = algaePose.minus(drivetrainPose);

            return 1/Math.min(comparingTransform.getTranslation().getNorm(),5);
        })).andThen(scoringMechanism.score(Positions.AlgaeN)));
    }

    public Command autoScoreProcessor(){
        return Commands.race(autoToProcessor()
        ,Commands.race(Commands.waitUntil(()->{
            Pose2d drivetrainPose = drivetrain.getPose();
            Pose2d processorPose = GameData.processorPose(DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red);
            Transform2d comparingTransform = processorPose.minus(drivetrainPose);

            return (comparingTransform.getTranslation().getNorm() < positionalToleranceMeters) 
                && (processorPose.getRotation().getRotations() - drivetrainPose.getRotation().getRotations() < rotationalToleranceRotations);
        })
        ,scoringMechanism.preset(Positions.AlgaeP,()->{
            Pose2d drivetrainPose = drivetrain.getPose();
            Pose2d processorPose = GameData.processorPose(DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red);
            Transform2d comparingTransform = processorPose.minus(drivetrainPose);

            return 1/Math.min(comparingTransform.getTranslation().getNorm(),5);
        })).andThen(scoringMechanism.score(Positions.AlgaeP)));
    }

    public Command autoScoreAlgae(int level){
        return Commands.either(autoScoreProcessor(), autoScoreNet(), ()->level == 1);
    }

    public Command autoIntakeCoral(int place){
        return Commands.race(autoToFeeder(place), scoringMechanism.intake());
    }

    public Command autoIntakeCoral(int place, double rightOffset){
        return Commands.race(autoToFeeder(place, rightOffset), scoringMechanism.intake());
    }

    public Command autoIntakeAlgae(int place){
        return Commands.race(autoToAlgae(place), scoringMechanism.grabAlgae((place%2)+1));
    }
}
