// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;

import com.fasterxml.jackson.databind.ser.std.BooleanSerializer;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.constants.DriveBaseConstants;
import frc.robot.simulation.SimMotor;

public class DriveBase extends SubsystemBase {

  public class DriveState{
    public Pose2d drivePose = new Pose2d();
    public Pose2d cameraPose = new Pose2d();

    public List<Pose2d> cameraList = new ArrayList<>();
  }
  private DriveState driveState = new DriveState();

  private SimMotor leftLeader = new SimMotor(DriveBaseConstants.leftLeaderID);
  private SimMotor leftFollower = new SimMotor(DriveBaseConstants.leftfollowerID);

  private SimMotor rightLeader = new SimMotor(DriveBaseConstants.rightLeaderID);
  private SimMotor rightFollower = new SimMotor(DriveBaseConstants.rightFollowerID);

  private Consumer<DriveState> m_telemetryFunction = null;

  private PhotonVision mainCamera = new PhotonVision();

  /** Creates a new DriveBase. */
  public DriveBase() {
    leftLeader.addFollower(leftFollower);
    leftLeader.setInverted(DriveBaseConstants.leftInverted);
    rightLeader.addFollower(rightFollower);
    rightLeader.setInverted(DriveBaseConstants.rightInverted);

    if (Robot.isSimulation()){
      leftLeader.setFrictionVelocity(.1);
      rightLeader.setFrictionVelocity(.1);
      leftFollower.setFrictionVelocity(.1);
      rightFollower.setFrictionVelocity(.1);

      leftLeader.setMaxSpeed(5);
      leftFollower.setMaxSpeed(5);
      rightLeader.setMaxSpeed(5);
      rightFollower.setMaxSpeed(5);
    }
  }

  public void setPose(Pose2d pose){
    mainCamera.resetSimPose(pose);
    driveState.drivePose = pose;
  }

  public void arcadeDrive(double xSpeed, double zRotation, boolean squareInputs) {
    xSpeed = MathUtil.clamp(xSpeed, -1.0, 1.0);
    zRotation = -MathUtil.clamp(zRotation, -1.0, 1.0);
    // Square the inputs (while preserving the sign) to increase fine control
    // while permitting full power.

    if (squareInputs) {
      xSpeed = Math.copySign(xSpeed * xSpeed, xSpeed);
      zRotation = Math.copySign(zRotation * zRotation, zRotation);
    }

    double leftSpeed = xSpeed - zRotation;
    double rightSpeed = xSpeed + zRotation;

    // Find the maximum possible value of (throttle + turn) along the vector
    // that the joystick is pointing, then desaturate the wheel speeds
    double greaterInput = Math.max(Math.abs(xSpeed), Math.abs(zRotation));
    double lesserInput = Math.min(Math.abs(xSpeed), Math.abs(zRotation));

    if (greaterInput == 0){
      leftLeader.set(0);
      rightLeader.set(0);
    }

    double saturatedInput = (greaterInput + lesserInput) / greaterInput;
    leftSpeed /= saturatedInput;
    rightSpeed /= saturatedInput;

    leftLeader.set(leftSpeed);
    rightLeader.set(rightSpeed);
  }

  public Command arcadeDriveCommand(DoubleSupplier xSpeed, DoubleSupplier zRotation, boolean squareInputs){
    return this.run(()->arcadeDrive(xSpeed.getAsDouble(), zRotation.getAsDouble(), squareInputs));
  }

  public PhotonVision getCamera(){
    return mainCamera;
  }

  @Override
  public void periodic() {
    updatePose();
    try {
      driveState.cameraList = mainCamera.getCurrentTargets();
      driveState.cameraPose = mainCamera.getEstimatedGlobalPose().get().estimatedPose.toPose2d();
    } catch (Exception e) {
      // TODO: handle exception
    }
    
    

    if (m_telemetryFunction!=null){
      try {
          m_telemetryFunction.accept(driveState);
      } catch (Exception e) {
          //AHH
      }
    }
    if (Robot.isSimulation()){
      mainCamera.simulationPeriodic(driveState.drivePose);
    }
  }


  private double lastLeftPosition = 0;
  private double lastRightPosition = 0;
  private void updatePose(){

    if (!Robot.isSimulation()){
      return;
    }
    double dl = (leftLeader.getSimPosition() - lastLeftPosition)*2*Math.PI*DriveBaseConstants.robotWheelRadius;
    double dr = (rightLeader.getSimPosition() - lastRightPosition)*2*Math.PI*DriveBaseConstants.robotWheelRadius;
    double dm = (dl+dr)/2;

    lastLeftPosition = leftLeader.getSimPosition();
    lastRightPosition = rightLeader.getSimPosition();

    double theta = (dr-dl)/DriveBaseConstants.robotLengthInches;

    double deltaY = Units.inchesToMeters(dm)*Math.sin(driveState.drivePose.getRotation().getRadians() + theta/2);
    double deltaX = Units.inchesToMeters(dm)*Math.cos(driveState.drivePose.getRotation().getRadians() + theta/2);

    driveState.drivePose = new Pose2d(driveState.drivePose.getX() + deltaX, driveState.drivePose.getY() + deltaY, Rotation2d.fromRadians(driveState.drivePose.getRotation().getRadians()+theta));
  }

  public void registerTelemetry(Consumer<DriveState> telemetryFunction) {
    m_telemetryFunction = telemetryFunction;
  }
}
