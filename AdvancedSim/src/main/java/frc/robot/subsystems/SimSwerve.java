package frc.robot.subsystems;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.gameConnection.GameConnection;
import frc.robot.gameConnection.RectangleCollision;
import frc.robot.gameConnection.Vector2;
import frc.robot.subsystems.swerve.SwerveBase;
import frc.robot.subsystems.swerve.SwerveModule;
import frc.robot.gameConnection.Rectangle;


public class SimSwerve extends SwerveBase{

    public SimSwerve(int gyroID, List<SwerveModule> list) {
        super(gyroID, list);
        GameConnection.assignDrivetrain(this);
    }

    public void handleSimCollision(Rectangle robot, Rectangle other){
        double[] finalVelocities = RectangleCollision.calculateCollision(robot, other);
        // Vector2 currentVelocity = Vector2.Reflect(new Vector2(speeds.x, speeds.y), normal);
        
        // double collisionTorque = Vector2.Dot(normal, currentVelocity);
        // double rotVel = rotationVelocity + collisionTorque;
        // normal.rotate(-90);
        if (finalVelocities == null){
            return;
        }
        System.out.println(finalVelocities[0]);
        ChassisSpeeds newSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(finalVelocities[0],finalVelocities[1],finalVelocities[2],driveState.drivePose.getRotation().minus(rotationOffset));
        SwerveModuleState[] newStates = kinematics.toSwerveModuleStates(newSpeeds);
        for (int i = 0; i < driveState.modules.size(); i++){
            driveState.modules.get(i).overrideActualState(newStates[i]);
        }
    }

    public void periodic(){
        GameConnection.setRobotPose(getPose());
        super.periodic();
        // for (int i = 0; i < driveState.modules.size(); i++){
        //     SwerveModuleState appliedState = driveState.modules.get(i).goalState;
        //     appliedState.speedMetersPerSecond *= appliedState.angle.minus(Rotation2d.fromRotations(driveState.modules.get(i).steerMotor.getFixedPosition())).getCos();
        //     driveState.modules.get(i).overrideActualState(appliedState);
        // }
    }

    public void handleSimCollision(Vector2 collisionData) {
        // Vector2 currentVelocity = Vector2.Reflect(new Vector2(speeds.x, speeds.y), normal);
        
        // double collisionTorque = Vector2.Dot(normal, currentVelocity);
        // double rotVel = rotationVelocity + collisionTorque;
        // normal.rotate(-90);
        ChassisSpeeds newSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(collisionData.x, collisionData.y,0,driveState.drivePose.getRotation().minus(rotationOffset));
        SwerveModuleState[] newStates = kinematics.toSwerveModuleStates(newSpeeds);
        for (int i = 0; i < driveState.modules.size(); i++){
            driveState.modules.get(i).overrideActualState(newStates[i]);
        }
    }
}
