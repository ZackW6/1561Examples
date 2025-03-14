// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands;

// import java.util.ArrayList;
// import java.util.Set;

// import com.ctre.phoenix.Util;
// import com.ctre.phoenix6.Utils;
// import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
// import com.ctre.phoenix6.swerve.SwerveRequest;

// import edu.wpi.first.math.util.Units;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.Subsystem;
// import frc.robot.generated.TunerConstants;
// import frc.robot.subsystems.swerve.SwerveDrive;
// import frc.robot.subsystems.swerve.realSwerve.CommandSwerveDrivetrain;

// public class WheelRadiusCommand extends Command {

//     private SwerveDrive drivetrain;
//     private CommandSwerveDrivetrain driveIO;
//     private double[] initPositions = new double[4];
//     private double initGyro;
//     private double driveBaseRadius;
//     private ArrayList<Double> lastGuesses = new ArrayList<>();
//     private boolean isFinished = false;

//     private double initTime;
//     private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
//       .withDeadband(0).withRotationalDeadband(0)
//       .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    
//     public WheelRadiusCommand(SwerveDrive drivetrain) {
//         this.drivetrain = drivetrain;
//         if (drivetrain.getDriveIO() instanceof CommandSwerveDrivetrain){
//             driveIO = (CommandSwerveDrivetrain)drivetrain.getDriveIO();
//         }else{
//             isFinished = true;
//         }
//         this.addRequirements(drivetrain);
//     }

//     private double wheelTravelledRotations(){
//         return (Math.abs(initPositions[0]-driveIO.getModule(0).getDriveMotor().getPosition().getValueAsDouble()/TunerConstants.kDriveGearRatio)
//         +Math.abs(initPositions[1] - driveIO.getModule(1).getDriveMotor().getPosition().getValueAsDouble()/TunerConstants.kDriveGearRatio)
//         +Math.abs(initPositions[2] - driveIO.getModule(2).getDriveMotor().getPosition().getValueAsDouble()/TunerConstants.kDriveGearRatio)
//         +Math.abs(initPositions[3] - driveIO.getModule(3).getDriveMotor().getPosition().getValueAsDouble()/TunerConstants.kDriveGearRatio))
//         /4;
//     }

//     private double getGyroRotations(){
//         return drivetrain.getYaw().getRotations();
//     }

//     // Called when the command is initially scheduled.
//     @Override
//     public void initialize() {
//         if (driveIO == null){
//             isFinished = true;
//             return;
//         }
//         initPositions[0] = driveIO.getModule(0).getDriveMotor().getPosition().getValueAsDouble()/TunerConstants.kDriveGearRatio;
//         initPositions[1] = driveIO.getModule(1).getDriveMotor().getPosition().getValueAsDouble()/TunerConstants.kDriveGearRatio;
//         initPositions[2] = driveIO.getModule(2).getDriveMotor().getPosition().getValueAsDouble()/TunerConstants.kDriveGearRatio;
//         initPositions[3] = driveIO.getModule(3).getDriveMotor().getPosition().getValueAsDouble()/TunerConstants.kDriveGearRatio;

//         initGyro = getGyroRotations();
//         driveBaseRadius = TunerConstants.driveBaseRadius;
//         initTime = Utils.getCurrentTimeSeconds();
//         isFinished = false;

//         driveIO.setControl(drive.withRotationalRate(Units.degreesToRadians(120)));
//     }

//     // Called every time the scheduler runs while the command is scheduled.
//     @Override
//     public void execute() {
//         if (driveIO == null){
//             isFinished = true;
//             return;
//         }
//         // System.out.println(Math.abs(initGyro-getGyroRotations())+"  "+wheelTravelledRotations());
//         if (initTime+55 <=Utils.getCurrentTimeSeconds()){
//             lastGuesses.add((Math.abs(initGyro-getGyroRotations())
//                 *driveBaseRadius*1.41484359/wheelTravelledRotations()));
//         }
//         if (initTime+60 <= Utils.getCurrentTimeSeconds()){
//             endProcedure();
//         }
//     }

//     public void endProcedure(){
//         System.out.println(Math.abs(initGyro-getGyroRotations())+"  "+wheelTravelledRotations());
//         double finalGuess = 0;
//         for (double i : lastGuesses){
//             finalGuess+=i;
//         }
//         finalGuess/=lastGuesses.size();
//         System.out.println("Wheel Radius Inches: " +
//         "\n"+
//         lastGuesses.size()+
//         "\n"+
//         wheelTravelledRotations()+//Units.degreesToRadians(drivetrain.getPigeon2().getYaw().getValueAsDouble())+
//         // Math.abs(initPosition-Units.rotationsToRadians(drivetrain.getModule(0).getDriveMotor().getPosition().getValueAsDouble()))+
//         "\n"+
//         (finalGuess));
//         driveIO.setControl(drive.withRotationalRate(Units.degreesToRadians(0)));
//         lastGuesses.clear();
//         isFinished = true;
//     }

//     // Called once the command ends or is interrupted.
//     @Override
//     public void end(boolean interrupted) {
        
//     }

//     // Returns true when the command should end.
//     @Override
//     public boolean isFinished() {
//         return isFinished;
//     }
// }