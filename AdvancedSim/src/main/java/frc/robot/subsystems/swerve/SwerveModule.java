package frc.robot.subsystems.swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.motors.BaseMotor;
import frc.robot.subsystems.motors.sim.SimMotor;

public class SwerveModule extends SubsystemBase{
    public SwerveModuleState goalState = new SwerveModuleState(0, Rotation2d.fromDegrees(0));
    public SwerveModuleState actualState = new SwerveModuleState(0, Rotation2d.fromDegrees(0));
    public SwerveModulePosition position = new SwerveModulePosition();

    // PIDController velocityController = new PIDController(10, 0, 0);
    // PIDController positionController = new PIDController(5, 0, 0.007);
    PIDController velocityController = new PIDController(.1, 0, 0);
    double driveK = 0;
    PIDController positionController = new PIDController(2, 0, 0.001);
    double steerK = 0;
    
    private final int steerMotorID;
    private final int steerEncoderID;
    private final int driveMotorID;
    private final int driveEncoderID;

    public double wheelRadiusInches;

    public final Translation2d tansform;

    public final BaseMotor steerMotor;
    public final BaseMotor driveMotor;

    public SwerveModule(BaseMotor drive, BaseMotor steer, Translation2d positionRobotRelative, double wheelRadiusInches){
      this.steerMotor = steer;
      this.driveMotor = drive;

      this.tansform = positionRobotRelative;

      this.steerMotorID = steer.getMotorID();
      this.driveMotorID = drive.getMotorID();

      this.steerEncoderID = steer.getEncoderID();
      this.driveEncoderID = drive.getEncoderID();
      this.wheelRadiusInches = wheelRadiusInches;
    }

    public void setGoal(SwerveModuleState goalState){
      this.goalState = goalState;//SwerveModuleState.optimize(goalState, Rotation2d.fromRotations(steerMotor.getFixedPosition()));
    }

    @Override
    public void periodic() {
        goalState = SwerveModuleState.optimize(goalState, Rotation2d.fromRotations(steerMotor.getFixedPosition()));
        SwerveModuleState appliedState = goalState;
        appliedState.speedMetersPerSecond *= appliedState.angle.minus(Rotation2d.fromRotations(steerMotor.getFixedPosition())).getCos();

        // System.out.println(driveMotor.getVelocity()+"   "+appliedState.speedMetersPerSecond/(2*Math.PI*Units.inchesToMeters(wheelRadiusInches)));
        // System.out.println(driveMotor.getVelocity()+"   "+appliedState.speedMetersPerSecond/(2*Math.PI*Units.inchesToMeters(wheelRadiusInches))+"   "+velocityController.calculate(driveMotor.getVelocity(), appliedState.speedMetersPerSecond/(2*Math.PI*Units.inchesToMeters(wheelRadiusInches))));
        double velocityInput = velocityController.calculate(driveMotor.getVelocity(), appliedState.speedMetersPerSecond/(2*Math.PI*Units.inchesToMeters(wheelRadiusInches)));
        driveMotor.set(velocityInput + (velocityInput < 0 ? -driveK: driveK));
        double steerInput = positionController.calculate(distToCorrectedPoint(steerMotor.getFixedPosition(), appliedState.angle.getRotations()),0);
        steerMotor.set(steerInput + (steerInput < 0 ? -steerK: steerK));

        this.actualState.angle = Rotation2d.fromRotations(this.steerMotor.getFixedPosition());
        this.actualState.speedMetersPerSecond = this.driveMotor.getVelocity()*2*Math.PI*Units.inchesToMeters(this.wheelRadiusInches);

        position.angle = Rotation2d.fromRotations(steerMotor.getFixedPosition());
        position.distanceMeters = Units.inchesToMeters(driveMotor.getPosition()*2*Math.PI*wheelRadiusInches);
    }

    /**
     * both points in the form from -.5 to .5
     * @param pos
     * @param goal
     * @return
     */
    private double distToCorrectedPoint(double pos, double goal){
        if (Math.abs(goal-pos) < .5){
            return goal-pos;
        }
        if (pos < 0){
            return goal-1-pos;
        }else{
            return goal+1-pos;
        }
    }

    public SwerveModule withSteerPID(double P, double I, double D){
        this.positionController = new PIDController(P, I, D);
        return this;
    }

    public SwerveModule withDrivePID(double P, double I, double D){
        this.velocityController = new PIDController(P, I, D);
        return this;
    }

    public SwerveModule withDriveK(double K){
        this.driveK = K;
        return this;
    }

    public SwerveModule withSteerK(double K){
        this.driveK = K;
        return this;
    }
  }
