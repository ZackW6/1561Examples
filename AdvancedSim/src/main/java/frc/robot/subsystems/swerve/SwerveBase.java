// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.fasterxml.jackson.databind.JsonSerializable.Base;
import com.fasterxml.jackson.databind.ser.std.BooleanSerializer;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.constants.DriveBaseConstants;
import frc.robot.gameConnection.GameConnection;
import frc.robot.subsystems.motors.BaseMotor;

public abstract class SwerveBase extends SubsystemBase {

  protected final SwerveDriveKinematics kinematics;
  protected final SwerveDriveOdometry odometry;

  protected Rotation2d rotationOffset = new Rotation2d();

  protected double time = Timer.getFPGATimestamp();
  protected Pigeon2 gyro;

  protected DriveState driveState = new DriveState();

  protected Consumer<DriveState> m_telemetryFunction = null;

  protected Translation2d[] moduleTranslations;

  public class DriveState{
    public Pose2d drivePose = new Pose2d();
    public List<SwerveModule> modules;
    public SwerveModulePosition[] modulePositions;
  }
  
  /**
   * in format
   * left front, right front, left back, right back
   */
  public SwerveBase(int gyroID, List<SwerveModule> list){
    gyro = new Pigeon2(gyroID, "Canivore");
    driveState.modules = list;
    Translation2d[] translations = new Translation2d[list.size()];
    SwerveModulePosition[] modulePositions = new SwerveModulePosition[list.size()];

    for (int i = 0; i < list.size(); i++){
      translations[i] = list.get(i).tansform;
      modulePositions[i] = list.get(i).position;
    }
    moduleTranslations = translations;


    kinematics = new SwerveDriveKinematics(translations);

    odometry = new SwerveDriveOdometry(
      kinematics, gyro.getRotation2d(),
      modulePositions, new Pose2d(0, 0, new Rotation2d()));
  }

  public Command fieldRelativeDrive(DoubleSupplier xAxis, DoubleSupplier yAxis, DoubleSupplier rotation){
    return Commands.run(()->setSwerveStates(xAxis.getAsDouble(), yAxis.getAsDouble(), rotation.getAsDouble(), this.rotationOffset),this);
  }

  /**
   * @param x
   * @param y
   * @param rot
   */
  public void setSwerveStates(double x, double y, double rot, Rotation2d offset){
    ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(x,y,rot,driveState.drivePose.getRotation().minus(offset));

    SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(speeds);

    int i = 0;
    for (SwerveModule module : driveState.modules){
      module.setGoal(moduleStates[i]);
      i++;
    }
  }

  public void registerTelemetry(Consumer<DriveState> telemetryFunction) {
    m_telemetryFunction = telemetryFunction;
  }

  public SwerveModuleState[] getModuleStates(){
    SwerveModuleState[] list = new SwerveModuleState[driveState.modules.size()];
    for (int i = 0; i < list.length; i++){
      list[i] = driveState.modules.get(i).actualState;
    }
    return list;
  }

  public SwerveModulePosition[] getModulePositions(){
    SwerveModulePosition[] list = new SwerveModulePosition[driveState.modules.size()];
    for (int i = 0; i < list.length; i++){
      list[i] = driveState.modules.get(i).position;
    }
    return list;
  }

  public Translation2d[] getModuleTranslations(){
    return moduleTranslations;
  }

  public ChassisSpeeds getChassisSpeeds(){
    return kinematics.toChassisSpeeds(getModuleStates());
  }

  public Pose2d getPose(){
    return odometry.getPoseMeters();
  }

  /**
   * Takes the specified location and makes it the current pose for
   * field-relative maneuvers
   *
   * @param location Pose to make the current pose at.
   */
  public void seedFieldRelative(Pose2d location) {
    odometry.resetPosition(gyro.getRotation2d(), getModulePositions(), location);
  }

  public void seedFieldRelative() {
    this.rotationOffset = driveState.drivePose.getRotation();
  }

  @Override
  public void periodic() {
    if (Robot.isSimulation()){
      
      double deltaTime = Timer.getFPGATimestamp() - time;
      time = Timer.getFPGATimestamp();

      double rotSpeed = kinematics.toChassisSpeeds(getModuleStates()).omegaRadiansPerSecond;
      gyro.getSimState().setRawYaw(gyro.getRotation2d().plus(Rotation2d.fromRadians(rotSpeed * deltaTime)).getDegrees());
    }

    driveState.drivePose = odometry.update(gyro.getRotation2d(),
      getModulePositions());

    if (m_telemetryFunction!=null){
      m_telemetryFunction.accept(driveState);
    }
  }

  public void configurePathPlanner() {
    double driveBaseRadius = DriveBaseConstants.driveBaseRadiusMeters;
    for (var moduleLocation : getModuleTranslations()) {
        driveBaseRadius = Math.max(driveBaseRadius, moduleLocation.getNorm());
    }

    AutoBuilder.configureHolonomic(
        ()->this.getPose(), // getState of the robot pose
        this::seedFieldRelative,  // Consumer for seeding pose against auto
        this::getChassisSpeeds,
        (speeds)->this.pathplannerDrive(speeds), // Consumer of ChassisSpeeds to drive the robot
        new HolonomicPathFollowerConfig(new PIDConstants(8, 0, 0),
                                        new PIDConstants(8, 0, 0),
                5.3,
        driveBaseRadius,
        new ReplanningConfig()),
        () -> {
            // Boolean supplier that controls when the path will be mirrored for the red alliance
            // This will flip the path being followed to the red side of the field.
            // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
            
            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
            }
            return false;
        },
        // Reference to this subsystem to set requirements // Change this if the path needs to be flipped on red vs blue
        this); // Subsystem for requirements
    }

    public void pathplannerDrive(ChassisSpeeds speeds){
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
            setSwerveStates(-speeds.vxMetersPerSecond, -speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond, new Rotation2d());
            return;
        }
                    
        setSwerveStates(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond, new Rotation2d());
    }

    public SwerveDriveKinematics getKinematics(){
      return kinematics;
    }

    public SwerveDriveOdometry getOdometry(){
      return odometry;
    }

    public void setGyroSim(Rotation2d rot){
      gyro.getSimState().setRawYaw(rot.getDegrees());
    }

    public Rotation2d getRotation2d(){
      return gyro.getRotation2d();
    }
}
