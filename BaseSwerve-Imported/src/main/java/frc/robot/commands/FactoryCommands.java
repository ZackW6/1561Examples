package frc.robot.commands;

import java.util.Optional;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.swerveHelpers.Vector2;
import frc.robot.util.PoseEX;

public class FactoryCommands {

    public CommandSwerveDrivetrain drivetrain;

    private static FactoryCommands instance;

    public FactoryCommands(CommandSwerveDrivetrain drivetrain){
        if (instance == null){
            instance = this;
        }
        this.drivetrain = drivetrain;
    }

    public static Optional<FactoryCommands> getInstance(){
        if (instance == null){
            return null;
        }
        return Optional.of(instance);
    }

    private final PIDController speedsPID = new PIDController(5, 0, 0);
    private final PIDController rotationPID = new PIDController(5, 0, 0);
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
                drivetrain.addRobotRelativeSpeeds(0,
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
     * Only moves robotRelative, toward the piece, and as soon as acknowledged will stop leading
     * @param pose
     * @param strengthCompWithNormalDrive
     * @param minDist
     * @param maxAngle at what angle toward the piece must your velocities be to react
     * @return
     */
    public Command passiveTowardPiece(Pose2d pose, double speedCap, double radianPSCap, double minDist, Rotation2d maxAngle, String key){
        
        return Commands.run(()->{

            Vector2 vector = new Vector2(pose.getX()-drivetrain.getPose().getX(), pose.getY()-drivetrain.getPose().getY()).rotate(-drivetrain.getYaw().getRadians());
            if (vector.getMagnitude() > minDist){
                drivetrain.addRobotRelativeSpeeds(0,
                    0,
                    0,
                    key);
                return;
            }

            Vector2 finalSpeeds = new Vector2(0,speedsPID.calculate(vector.y,0)*2);
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
     * @param strengthCompWithNormalDrive
     * @param minDist
     * @param maxAngle
     * @return
     */
    public Command activePointPose(Pose2d pose, double radianPSCap, double minDist, String key){
        return Commands.run(()->{
            Vector2 vec = new Vector2(pose.getX() - drivetrain.getPose().getX(), pose.getY() - drivetrain.getPose().getY());
            if (vec.getMagnitude() > minDist){
                drivetrain.addFieldRelativeSpeeds(0,
                    0,
                    0,
                    key);
                return;
            }
            double finalRotation = Math.min(Math.max(-rotationPID.calculate(PoseEX.getYawFromPose(drivetrain.getPose(), pose).getRadians(),0),-radianPSCap),radianPSCap);
            
            drivetrain.addFieldRelativeSpeeds(0,
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
}
