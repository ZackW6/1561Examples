package frc.robot.commands;

import java.util.Optional;
import java.util.Set;
import java.util.function.BooleanSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.PathOnTheFly.PathConfig;
import frc.robot.constants.GameData;
import frc.robot.subsystems.MainMechanism;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.subsystems.swerve.realSwerve.CommandSwerveDrivetrain;
import frc.robot.subsystems.swerve.swerveHelpers.Vector2;
import frc.robot.util.PoseEX;

public class FactoryCommands {

    private static final double positionalToleranceMeters = .05;
    private static final double rotationalToleranceRotations = .05;

    public SwerveDrive drivetrain;

    public MainMechanism scoringMechanism;

    private static FactoryCommands instance;

    private static final BooleanSupplier isRed = ()->DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red;

    public FactoryCommands(SwerveDrive drivetrain, MainMechanism scoringMechanism){
        if (instance == null){
            instance = this;
        }
        this.drivetrain = drivetrain;
        this.scoringMechanism = scoringMechanism;
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
    public Command passiveTowardPose(Pose2d pose, double speedCap, double radianPSCap, double minDist, String key){
        return Commands.run(()->{

            Vector2 vector = new Vector2(pose.getX()-drivetrain.getPose().getX(), pose.getY()-drivetrain.getPose().getY());
            if (vector.getMagnitude() > minDist){
                drivetrain.addFieldRelativeSpeeds(0,
                    0,
                    0,
                    key);
                return;
            }

            Vector2 finalSpeeds = new Vector2(-speedsPID.calculate(vector.x,0),-speedsPID.calculate(vector.y,0));
            if (finalSpeeds.getMagnitude() > speedCap){
                finalSpeeds = finalSpeeds.normalize().multiply(speedCap);
            }

            double finalRotation = Math.min(Math.max(rotationPID.calculate(distToCorrectedPoint(pose.getRotation().getRadians(), drivetrain.getPose().getRotation().getRadians()),0),-radianPSCap),radianPSCap);

            drivetrain.addFieldRelativeSpeeds(finalSpeeds.x,
                finalSpeeds.y,
                finalRotation,
                key);

        }).finallyDo(()->{
            drivetrain.removeSource(key);
        });
    }

    /**
     * @param translation
     * @param strengthCompWithNormalDrive
     * @param minDist
     * @param maxAngle
     * @return
     */
    public Command passiveTowardTranslation(Translation2d translation, double speedCap, double radianPSCap, double minDist, String key){
        return Commands.run(()->{

            Vector2 vector = new Vector2(translation.getX()-drivetrain.getPose().getX(), translation.getY()-drivetrain.getPose().getY());
            if (vector.getMagnitude() > minDist){
                drivetrain.addFieldRelativeSpeeds(0,
                    0,
                    0,
                    key);
                return;
            }

            Vector2 finalSpeeds = new Vector2(-speedsPID.calculate(vector.x,0),-speedsPID.calculate(vector.y,0));
            if (finalSpeeds.getMagnitude() > speedCap){
                finalSpeeds = finalSpeeds.normalize().multiply(speedCap);
            }


            drivetrain.addFieldRelativeSpeeds(finalSpeeds.x,
                finalSpeeds.y,
                0,
                key);

        }).finallyDo(()->{
            drivetrain.removeSource(key);
        });
    }

    /**
     * Only moves robotRelative, toward the piece, and as soon as acknowledged will stop leading
     * @param pose
     * @param strengthCompWithNormalDrive
     * @param minDist
     * @param key
     * @return 
     */
    public Command passiveTowardPiece(Pose2d pose, double speedCap, double radianPSCap, double minDist, String key){
        
        return Commands.run(()->{

            Vector2 vector = new Vector2(pose.getX()-drivetrain.getPose().getX(), pose.getY()-drivetrain.getPose().getY()).rotate(-drivetrain.getYaw().getRadians());
            if (vector.getMagnitude() > minDist){
                drivetrain.addRobotRelativeSpeeds(0,
                    0,
                    0,
                    key);
                return;
            }

            Vector2 finalSpeeds = new Vector2(0, speedsPID.calculate(vector.y,0)*2);
            if (finalSpeeds.getMagnitude() > speedCap){
                finalSpeeds = finalSpeeds.normalize().multiply(speedCap);
            }
            
            drivetrain.addRobotRelativeSpeeds(0,
                -finalSpeeds.y,
                0,
                key);

        }).finallyDo(()->{
            drivetrain.removeSource(key);
        });
    }

    /**
     * @param pose
     * @param radianPSCap
     * @param minDist
     * @param key
     * @return
     */
    public Command activePointPose(Pose2d pose, double radianPSCap, double minDist, String key){
        return Commands.run(()->{
            Vector2 vec = new Vector2(pose.getX() - drivetrain.getPose().getX(), pose.getY() - drivetrain.getPose().getY());
            if (vec.getMagnitude() > minDist){
                drivetrain.addRobotRelativeSpeeds(0,
                    0,
                    0,
                    key);
                return;
            }
            double finalRotation = Math.min(Math.max(-rotationPID.calculate(PoseEX.getYawFromPose(drivetrain.getPose(), pose).getRadians(),0),-radianPSCap),radianPSCap);
            drivetrain.addRobotRelativeSpeeds(0,
                0,
                finalRotation,
                key);

        }).finallyDo(()->{
            drivetrain.removeSource(key);
        });
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
            .andThen(passiveTowardPose(pose, 5, 3*Math.PI, 5, "toPose"+pose));
    }

    /**
     * does not end automatically, must be ended.
     * In this case, pointing is only added lightly.
     * @return
     */
    public Command toPoseWhilePointing(Pose2d pose){
        return PathOnTheFly.AutoToPoint.getToPoint(pose, new PathConfig(5,5,
            Rotation2d.fromRadians(3*Math.PI),Rotation2d.fromDegrees(720),0,0))
            .andThen(passiveTowardPose(new Pose2d(pose.getX(), pose.getY(), pose.getRotation()), 5, 3*Math.PI, 2, "toPoseWP"+pose))
            .alongWith(activePointPose(pose, 1*Math.PI, 18, "toPoseWPPoint"+pose));
    }

    public Command autoToCoral(int place){
        int clampedNum = Math.max(Math.min(place,12),1);
        return Commands.defer(()->toPose(GameData.coralPose(clampedNum, DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red)
        ,1.2),Set.of());
    }

    public Command autoToFeeder(int place){
        int clampedNum = Math.max(Math.min(place,2),1);
        return Commands.defer(()->toPose(GameData.feederPose(clampedNum, DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red)
        ,3),Set.of());
    }

    public Command autoToProcessor(){
        return Commands.defer(()->toPose(GameData.processorPose(DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red)
        ,3),Set.of());
    }

    public Command autoScoreCoral(int place, int level){
        return Commands.race(autoToCoral(place)
        ,Commands.race(Commands.waitUntil(()->{
            Pose2d drivetrainPose = drivetrain.getPose();
            Pose2d coralPose = GameData.coralPose(place, DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red);
            Transform2d comparingTransform = coralPose.minus(drivetrainPose);

            return (comparingTransform.getTranslation().getNorm() < positionalToleranceMeters) 
                && (coralPose.getRotation().getRotations() - drivetrainPose.getRotation().getRotations() < rotationalToleranceRotations);
        })
        ,scoringMechanism.preset(level,()->{
            Pose2d drivetrainPose = drivetrain.getPose();
            Pose2d coralPose = GameData.coralPose(place, DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red);
            Transform2d comparingTransform = coralPose.minus(drivetrainPose);

            return 1/Math.min(comparingTransform.getTranslation().getNorm(),5);
        })).andThen(scoringMechanism.score(level)));
    }

    public Command autoIntakeCoral(int place){
        //TODO TIMEOUT IS ONLY FOR TESTING
        return Commands.race(autoToFeeder(place), scoringMechanism.intake());
    }
}
