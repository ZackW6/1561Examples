// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import java.util.List;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.motors.sim.SimMotor;
import frc.robot.subsystems.motors.talon.TalonMotor;
import frc.robot.subsystems.swerve.SwerveBase;
import frc.robot.subsystems.swerve.SwerveModule;

/** Add your docs here. */
public class DriveBaseConstants {
    public static final double speedAt12Volts = 5;
    public static final int gyroID = 0;

    public static final int frontLeftSteerID = 1;
    public static final int frontLeftEncoderID = 1;
    public static final double frontLeftSteerOffset = 0.066162109375;

    public static final int frontLeftDriveID = 2;

    public static final int frontRightSteerID = 3;
    public static final int frontRightEncoderID = 2;
    public static final double frontRightSteerOffset = -0.259033203125;

    public static final int frontRightDriveID = 4;

    public static final int backLeftSteerID = 5;
    public static final int backLeftEncoderID = 3;
    public static final double backLeftSteerOffset = -0.30859375;

    public static final int backLeftDriveID = 6;

    public static final int backRightSteerID = 7;
    public static final int backRightEncoderID = 4;
    public static final double backRightSteerOffset = 0.4443359375;

    public static final int backRightDriveID = 8;


    public static final double wheelRadiusInches = 1.935;
    public static final double driveBaseRadiusInches = 9.375;

    public static final double driveBaseRadiusMeters = Units.inchesToMeters(driveBaseRadiusInches);

    public static final double robotWeightPounds = 70;

    public class TalonMotorConstants{
        public static final double driveGearRatio = 6.746031746031747;
        public static final double steerGearRatio = 21.428571428571427;
    }

    public class SimMotorConstants{
        public static final double driveFrictionCoef = 0.25;
        public static final double driveJKG = 1;

        public static final double steerFrictionCoef = 0.25;
        public static final double steerJKG = 0.00001;
    }

    /**
     * FL, FR, BL, BR if applicable
     * x is forward, y is positive left
     */
    // public static final SwerveDrive simDrive = new SwerveDrive(0, List.of(
    //     new SwerveModule(new SimMotor(frontLeftDriveID, TalonMotorConstants.driveGearRatio, SimMotorConstants.driveJKG, SimMotorConstants.driveFrictionCoef, DCMotor.getKrakenX60(1))
    //     , new SimMotor(frontLeftSteerID, TalonMotorConstants.steerGearRatio, SimMotorConstants.steerJKG, SimMotorConstants.steerFrictionCoef, DCMotor.getKrakenX60(1))
    //     , new Translation2d(.5,.5),wheelRadiusInches),

    //     new SwerveModule(new SimMotor(frontRightDriveID, TalonMotorConstants.driveGearRatio, SimMotorConstants.driveJKG, SimMotorConstants.driveFrictionCoef, DCMotor.getKrakenX60(1))
    //     , new SimMotor(frontRightSteerID, TalonMotorConstants.steerGearRatio, SimMotorConstants.steerJKG, SimMotorConstants.steerFrictionCoef, DCMotor.getKrakenX60(1))
    //     , new Translation2d(.5,-.5),wheelRadiusInches),

    //     new SwerveModule(new SimMotor(backLeftDriveID, TalonMotorConstants.driveGearRatio, SimMotorConstants.driveJKG, SimMotorConstants.driveFrictionCoef, DCMotor.getKrakenX60(1))
    //     , new SimMotor(backLeftSteerID, TalonMotorConstants.steerGearRatio, SimMotorConstants.steerJKG, SimMotorConstants.steerFrictionCoef, DCMotor.getKrakenX60(1))
    //     , new Translation2d(-.5,.5),wheelRadiusInches),

    //     new SwerveModule(new SimMotor(backRightDriveID, TalonMotorConstants.driveGearRatio, SimMotorConstants.driveJKG, SimMotorConstants.driveFrictionCoef, DCMotor.getKrakenX60(1))
    //     , new SimMotor(backRightSteerID, TalonMotorConstants.steerGearRatio, SimMotorConstants.steerJKG, SimMotorConstants.steerFrictionCoef, DCMotor.getKrakenX60(1))
    //     , new Translation2d(-.5,-.5),wheelRadiusInches)
    // ));

    public static final SwerveDrive talonDrive = new SwerveDrive(0, List.of(
        new SwerveModule(new TalonMotor(frontLeftDriveID,TalonMotorConstants.driveGearRatio)
        , new TalonMotor(frontLeftSteerID, frontLeftEncoderID, 1,TalonMotorConstants.steerGearRatio, frontLeftSteerOffset)
        , new Translation2d(driveBaseRadiusMeters,driveBaseRadiusMeters),wheelRadiusInches),

        new SwerveModule(new TalonMotor(frontRightDriveID,TalonMotorConstants.driveGearRatio).setInvert(true)
        , new TalonMotor(frontRightSteerID, frontRightEncoderID, 1,TalonMotorConstants.steerGearRatio, frontRightSteerOffset)
        , new Translation2d(driveBaseRadiusMeters,-driveBaseRadiusMeters),wheelRadiusInches),

        new SwerveModule(new TalonMotor(backLeftDriveID,TalonMotorConstants.driveGearRatio)
        , new TalonMotor(backLeftSteerID, backLeftEncoderID, 1,TalonMotorConstants.steerGearRatio, backLeftSteerOffset)
        , new Translation2d(-driveBaseRadiusMeters,driveBaseRadiusMeters),wheelRadiusInches),

        new SwerveModule(new TalonMotor(backRightDriveID,TalonMotorConstants.driveGearRatio).setInvert(true)
        , new TalonMotor(backRightSteerID, backRightEncoderID, 1,TalonMotorConstants.steerGearRatio, backRightSteerOffset)
        , new Translation2d(-driveBaseRadiusMeters,-driveBaseRadiusMeters),wheelRadiusInches)
    ));
}
