package frc.robot.subsystems;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.constants.DriveBaseConstants;
import frc.robot.subsystems.swerve.SwerveBase;
import frc.robot.subsystems.swerve.SwerveModule;

public class SwerveDrive extends SwerveBase{

    public SwerveDrive(int gyroID, List<SwerveModule> list) {
        super(gyroID, list);
    }
}
