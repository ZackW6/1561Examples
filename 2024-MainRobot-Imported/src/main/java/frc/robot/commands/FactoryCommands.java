// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Currency;
import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentric;
import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentricFacingAngle;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import com.ctre.phoenix6.swerve.SwerveRequest.RobotCentric;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.CommandSwerveDrivetrain;
import frc.robot.commands.FactoryCommands.State;
import frc.robot.commands.PathOnTheFly.AutoToPoint;
import frc.robot.commands.PathOnTheFly.PathConfig;
import frc.robot.constants.LimelightConstants;
import frc.robot.constants.ShooterConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Arm;
// import frc.robot.subsystems.Candle;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.ObjectDetection;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Arm.ArmState;
import frc.robot.util.PoseEX;

public class FactoryCommands extends SubsystemBase{
  private Arm arm;
  private Shooter shooter;
  private Intake intake;
  // private Candle candle;
  private CommandSwerveDrivetrain drivetrain;
  private CommandXboxController xboxController;
  private ObjectDetection limelightObject;

  private Supplier<Pose2d> speakerPose = ()->{
    if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get().equals(Alliance.Red)){
      return LimelightConstants.K_TAG_LAYOUT.getTagPose(4).get().toPose2d();
    }
    return LimelightConstants.K_TAG_LAYOUT.getTagPose(7).get().toPose2d();
  };

  public FactoryCommands(Arm arm, Shooter shooter, Intake intake, CommandSwerveDrivetrain drivetrain, ObjectDetection limelightCam, CommandXboxController xboxController){
    this.arm = arm;
    this.shooter = shooter;
    this.intake = intake;
    // this.candle = candle;
    this.drivetrain = drivetrain;
    this.xboxController = xboxController;
    this.limelightObject = limelightCam;
  }

  public Command shoot() {
    return Commands.either(ampShot(), speakerShoot(60,80), ()->(getState() == State.Amp));
  }

  public Command align() {
    return Commands.either(alignToAmp(), getToSpeakerCommand(), ()->(getState() == State.Amp));
  }

  public Command speakerShoot(double LSpeed, double RSpeed){
    return Commands.deadline(switchState(State.Speaker)
      .andThen(Commands.waitSeconds(.01)).andThen(Commands.waitUntil(() ->shooter.isShooterAtTargetSpeed()))
      .andThen(Commands.waitUntil(()->arm.isArmAtAngle())).andThen(intake.outtakePiece()).andThen(intake.stop())
      ,shooter.shootVelocity(LSpeed, RSpeed));
  }

  public Command ampShot(){
    return Commands.deadline(Commands.waitSeconds(.5)
      ,Commands.waitSeconds(0.003/*0.006*/).andThen(arm.setArmRotation(ArmState.AmpMove))
      ,intake.setVelocity(-18.6/*-19.25*/)).andThen(switchState(State.Speaker));
  }

  public Command intake(){
    return Commands.deadline(intake.intakePiece(),switchState(State.Intake))
      // .andThen(candle.pickUpLights())
      // .andThen(Commands.runOnce(()->xboxController.getHID().setRumble(RumbleType.kBothRumble, 1)))
      .andThen(Commands.deadline(switchState(State.Speaker).andThen(Commands.waitSeconds(.5)).andThen(Commands.waitUntil(()->arm.isArmAtAngle()))
      ,intake.setVelocity(15)))
      .handleInterrupt(()->{
        // xboxController.getHID().setRumble(RumbleType.kBothRumble, 0);
        Command switchEnd = switchState(State.Speaker);
        switchEnd.initialize();
        switchEnd.schedule();
        intake.stop();
        // candle.idleLED();
      }).withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
  }

  public Command intakeMainAuto(){
    return Commands.deadline(intake.intakePiece(),switchState(State.Intake))
      // .andThen(candle.pickUpLights())
      // .andThen(Commands.runOnce(()->xboxController.getHID().setRumble(RumbleType.kBothRumble, 1)))
      .andThen(Commands.deadline(switchState(State.Speaker).andThen(Commands.waitSeconds(.3)).andThen(Commands.waitUntil(()->arm.isArmAtAngle()))
      ,intake.setVelocity(15)))
      .handleInterrupt(()->{
        // xboxController.getHID().setRumble(RumbleType.kBothRumble, 0);
        Command switchEnd = switchState(State.Speaker);
        switchEnd.initialize();
        switchEnd.schedule();
        intake.stop();
        // candle.idleLED();
      }).withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
  }

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(0).withRotationalDeadband(0)
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);


  //Used to be FielCentricFacingRed
  private final FieldCentric ampDrive = new SwerveRequest.FieldCentric()
      .withDeadband(0).withRotationalDeadband(0)
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);


  private final PIDController thetaControllerAmp = new PIDController(6,0,.7);//(12.2,1.1,.4);
  private final PIDController ampHorizontal = new PIDController(6,0,.2);//(12.2,1.1,.4);
  private final PIDController ampVertical = new PIDController(4,0,.2);//(12.2,1.1,.4);

  Supplier<Pose2d> ampPose = ()->{
      if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get().equals(Alliance.Red)){
        return LimelightConstants.K_TAG_LAYOUT.getTagPose(5).get().toPose2d();
      }
      return LimelightConstants.K_TAG_LAYOUT.getTagPose(6).get().toPose2d();
    };
  public Command alignToAmp() {

    DoubleSupplier xAxis = ()-> ampHorizontal.calculate(drivetrain.getPose().getX(),ampPose.get().getX());

    DoubleSupplier yAxis = ()-> ampVertical.calculate(drivetrain.getPose().getY(),ampPose.get().getY()-.5);

    thetaControllerAmp.reset();
    DoubleSupplier rotationalVelocity = () -> thetaControllerAmp.calculate(correctYaw((drivetrain.getPose().getRotation().getDegrees())%360,90), 90);
    
    return AutoToPoint.getToPoint(new Pose2d(ampPose.get().getX(),ampPose.get().getY(),Rotation2d.fromDegrees(90))).until(()->PoseEX.getDistanceFromPoseMeters(drivetrain.getPose(), ampPose.get())<2)
    .andThen(drivetrain.applyRequest(() -> ampDrive.withVelocityX(xAxis.getAsDouble())
        .withCenterOfRotation(drivetrain.getPose().getTranslation())
        .withVelocityY(yAxis.getAsDouble())
        .withRotationalRate(Units.degreesToRadians(rotationalVelocity.getAsDouble()))));
  }

  private double correctYaw(double x, double setpoint){
    if (x>180+setpoint){
      x-=360;
    }else if(x<-180+setpoint){
      x+=360;
    }
    return x;
  }
  private final PIDController thetaControllerSpeaker = new PIDController(2,0,0.01);

  public Command alignToCorner() {
    DoubleSupplier xAxis = () -> -xboxController.getLeftY() * TunerConstants.kSpeedAt12VoltsMps;
    DoubleSupplier yAxis = () -> -xboxController.getLeftX() * TunerConstants.kSpeedAt12VoltsMps;
    thetaControllerSpeaker.reset();
    DoubleSupplier rotationalVelocity = () -> {
      if(drivetrain.getAngleFromCorner().getDegrees()>0){
        return -thetaControllerSpeaker.calculate(drivetrain.getAngleFromCorner().getDegrees()-180,0);//-7;
      }else{
        return -thetaControllerSpeaker.calculate(drivetrain.getAngleFromCorner().getDegrees()+180,0);//+7;
      }
    };
    return drivetrain.applyRequest(() -> drive.withVelocityX(xAxis.getAsDouble())
      .withVelocityY(yAxis.getAsDouble())
      .withRotationalRate(Units.degreesToRadians(rotationalVelocity.getAsDouble())));
  }

  private final PIDController thetaControllerPiece = new PIDController(5,0,0.2);

  private final PIDController distanceControllerPiece = new PIDController(2, 0, 0.2);
  private final PIDController horizontalControllerPiece = new PIDController(1, 0, .2);

  private final RobotCentric pieceDrive = new SwerveRequest.RobotCentric()
      .withDeadband(0).withRotationalDeadband(0)
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  public Command alignToPiece() {
    DoubleSupplier forwardSpeed = ()-> -distanceControllerPiece.calculate(limelightObject.getDistanceFromPieceVertical(), .5);

    DoubleSupplier leftSpeed = ()-> -horizontalControllerPiece.calculate(limelightObject.getDistanceFromPieceHorizontal(),0);
    
    DoubleSupplier rotationalVelocity = () -> -thetaControllerPiece.calculate(limelightObject.getHorizontalRotationFromPiece().getDegrees(),0);

    return drivetrain.applyRequest(() -> pieceDrive.withVelocityX(forwardSpeed.getAsDouble())
          .withVelocityY(leftSpeed.getAsDouble())
          .withRotationalRate(Units.degreesToRadians(rotationalVelocity.getAsDouble())));
  }

  private final FieldCentric speakerDrive = new FieldCentric()
      .withDeadband(0).withRotationalDeadband(0)
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
      .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance);
    
  private final PIDController distanceControllerSpeaker = new PIDController(.35, 0, 0);

  public Command getInRange() {
    DoubleSupplier distanceSpeed = ()-> -distanceControllerSpeaker.calculate(drivetrain.getDistanceFromPoseMeters(speakerPose.get()), 2.395);

    DoubleSupplier xAxis = ()->(speakerPose.get().getX()-drivetrain.getPose().getX())*distanceSpeed.getAsDouble();

    DoubleSupplier yAxis = ()->(speakerPose.get().getY()-drivetrain.getPose().getY())*distanceSpeed.getAsDouble();
    
    thetaControllerSpeaker.reset();

    DoubleSupplier rotationalVelocity = () -> {
      if(drivetrain.getAngleFromPose(speakerPose.get()).getDegrees()>0){
        return -thetaControllerSpeaker.calculate(drivetrain.getAngleFromPose(speakerPose.get()).getDegrees()-180,0);//-7;
      }else{
        return -thetaControllerSpeaker.calculate(drivetrain.getAngleFromPose(speakerPose.get()).getDegrees()+180,0);//+7;
      }
    };

    return drivetrain.applyRequest(() -> speakerDrive.withVelocityX(xAxis.getAsDouble()) 
        .withVelocityY(yAxis.getAsDouble())
        .withRotationalRate(Units.degreesToRadians(rotationalVelocity.getAsDouble())));
  }

  public Command getToPiecePoseCommand(){
    return getToPiecePoseCommand(new PathOnTheFly.PathConfig(5,5,Rotation2d.fromDegrees(720),Rotation2d.fromDegrees(720),0,0));
  }

  public Command getToPiecePoseCommand(PathConfig config){
    return Commands.either(getToPoseAndPoint(limelightObject.getPiecePose(), config)
      ,Commands.none(),()->limelightObject.isPiecePresent());
  }

  public Command getToPoseAndPoint(Pose2d pose, PathConfig config){
    return Commands.defer(()->
      AutoToPoint.getToPoint(PoseEX.getInbetweenPose2d(drivetrain.getPose(), pose, 0)
      .transformBy(new Transform2d(0,0,PoseEX.getPoseAngle(drivetrain.getPose(),pose))),config)
      .until(()->(PoseEX.getDistanceFromPoseMeters(drivetrain.getPose(), pose)<2.5)),Set.of(drivetrain))
      .andThen(Commands.defer(()->AutoToPoint.getToPoint(PoseEX.getInbetweenPose2d(drivetrain.getPose(), pose, .5)
      .transformBy(new Transform2d(0,0,PoseEX.getPoseAngle(drivetrain.getPose(),pose))),config),Set.of(drivetrain)));
  }

  public Command getToPieceCommand(){
    return Commands.either(Commands.defer(()->getToPiecePoseCommand(),Set.of(drivetrain))
      ,Commands.none(),()->limelightObject.isPiecePresent());
    // return Commands.either(Commands.defer(()->getToPiecePoseCommand().until(()->PoseEX.getDistanceFromPoseMeters(limelightObject.getPiecePose(),drivetrain.getPose())<1).andThen(alignToPiece()),Set.of(drivetrain))
    //   ,Commands.none(),()->limelightObject.isPiecePresent());
  }

  public DeferredCommand getToSpeakerCommand(){
    return new DeferredCommand(()->
      AutoToPoint.getToPoint(PoseEX.getInbetweenPose2d(drivetrain.getPose(),speakerPose.get(), 2)
      .transformBy(new Transform2d(0,0,PoseEX.getPoseAngle(drivetrain.getPose(),speakerPose.get())
      .plus(Rotation2d.fromDegrees(180)))),new PathOnTheFly.PathConfig(5,5,Rotation2d.fromDegrees(720),Rotation2d.fromDegrees(720),0,0))
      .until(()->PoseEX.getDistanceFromPoseMeters(speakerPose.get(),drivetrain.getPose())<4).andThen(getInRange()),Set.of(drivetrain));
  }

  public enum State{
    Amp,
    Speaker,
    Intake;
  }

  State state = State.Speaker;
  public Command switchState(State newState){
    switch (newState) {
        case Speaker:
            return arm.setArmDefault(ArmState.Speaker).alongWith(Commands.runOnce(()->state = State.Speaker));
        case Amp:
            return arm.setArmDefault(ArmState.Amp).alongWith(shooter.stopMotors()).alongWith(Commands.runOnce(()->state = State.Amp));
        case Intake:
            return arm.setArmDefault(ArmState.Intake).alongWith(Commands.runOnce(()->state = State.Intake));
    }
    return Commands.none();
  }

  public State getState(){
    return state;
  }

  double initPosition;
  double initGyro;
  double driveBaseRadius;
  public DeferredCommand findWheelRadius(){
    return new DeferredCommand(()->Commands.deadline(Commands.waitSeconds(60),
      Commands.runOnce(()->{
        initPosition = Units.rotationsToRadians(drivetrain.getModule(0).getDriveMotor().getPosition().getValueAsDouble());
        initGyro = Units.degreesToRadians(drivetrain.getPigeon2().getYaw().getValueAsDouble());
        driveBaseRadius = TunerConstants.driveBaseRadius;
      }).andThen(
      drivetrain.applyRequest(() -> drive.withVelocityX(0)
      .withVelocityY(0)
      .withRotationalRate(Units.degreesToRadians(360)))))
      .andThen(Commands.defer(()->Commands.print("Wheel Radius Inches: " +
      "\n"+
      Math.abs((Math.abs(initGyro)-Math.abs(Units.degreesToRadians(drivetrain.getPigeon2().getYaw().getValueAsDouble()))))+
      "\n"+
      Math.abs(Math.abs(initPosition)-Math.abs(Units.rotationsToRadians(drivetrain.getModule(0).getDriveMotor().getPosition().getValueAsDouble())))+
      "\n"+
      (Math.abs((Math.abs(initGyro)-Math.abs(Units.degreesToRadians(drivetrain.getPigeon2().getYaw().getValueAsDouble()))))
      *driveBaseRadius/Math.abs(Math.abs(initPosition)-Math.abs(Units.rotationsToRadians(drivetrain.getModule(0).getDriveMotor().getPosition().getValueAsDouble()))))
      ),Set.of()))
      ,Set.of(drivetrain));
  }

  public Command autoFindNote(){
    Command spin1 = drivetrain.applyRequest(() -> drive.withVelocityX(0)
      .withVelocityY(0)
      .withRotationalRate(Units.degreesToRadians(120))).until(()->(limelightObject.isPiecePresent() && limelightObject.getPiecePose().getX()<8.27));
    Command spin2 = drivetrain.applyRequest(() -> drive.withVelocityX(0)
      .withVelocityY(0)
      .withRotationalRate(Units.degreesToRadians(120))).until(()->(limelightObject.isPiecePresent() && limelightObject.getPiecePose().getX()>8.27));
    return Commands.either(spin1
      ,spin2
      , ()->DriverStation.getAlliance().isEmpty() || DriverStation.getAlliance().get() == Alliance.Blue)
      .andThen(Commands.defer(()->Commands.parallel(intakeMainAuto(), getToPiecePoseCommand()),Set.of()));
  }

  public void periodic(){

  }
}