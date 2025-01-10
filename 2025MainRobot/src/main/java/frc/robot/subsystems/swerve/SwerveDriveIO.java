package frc.robot.subsystems.swerve;

import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.ctre.phoenix6.swerve.jni.SwerveJNI;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.robot.constants.LimelightConstants;
import frc.robot.constants.PathplannerConstants;
import frc.robot.subsystems.swerve.swerveHelpers.MainDrive;
import frc.robot.subsystems.vision.Vision;

public interface SwerveDriveIO {

    public void setControl(SwerveRequest request);

    public Pose2d getPose();

    public void resetPose(Pose2d pose);

    public void seedFieldRelative(Rotation2d rot);

    public void addVisionMeasurement(Pose2d pose, double timestep, Vector<N3> stdDev);
    
    public void registerTelemetry(Consumer<SwerveDriveState> state);

    public Rotation2d getYaw();
    
    public Rotation2d getYawOffset();

    public ChassisSpeeds getSpeeds();

    public SwerveDriveState getState();
}
