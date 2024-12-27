package frc.robot.subsystems;

import java.util.List;
import java.util.Vector;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.proto.Kinematics;
import frc.robot.gameConnection.GameConnection;
import frc.robot.gameConnection.Vector2;
import frc.robot.gameConnection.GameConnection.GameData;
import frc.robot.gameConnection.GameConnection.GameInput;
import frc.robot.subsystems.swerve.SwerveBase;
import frc.robot.subsystems.swerve.SwerveModule;


public class SimSwerve extends SwerveBase{

    public SimSwerve(int gyroID, List<SwerveModule> list) {
        super(gyroID, list);
    }

    public void periodic(){
        super.periodic();
        GameConnection.drivePeriodic();
    }

    @Override
    public void seedFieldRelative(Pose2d pose) {
        super.seedFieldRelative(pose);
        if (GameConnection.isConnected()){
            GameConnection.seedFieldRelative(pose);
        }
    }
}
