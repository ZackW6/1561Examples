// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Function;
import java.util.stream.Stream;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.WaitAutos;
import frc.robot.commands.FactoryCommands;
import frc.robot.commands.WaitAutos.BranchInstruction;
import frc.robot.commands.WaitAutos.BranchInstruction.BeginPose;
import frc.robot.commands.WaitAutos.BranchInstruction.IntakePose;
import frc.robot.commands.WaitAutos.BranchInstruction.ShootPose;
import frc.robot.constants.GameData;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.ClimbMechanism;
import frc.robot.subsystems.MainMechanism;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.climb.Climber;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.ramp.Ramp;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.util.CustomController;
// import frc.robot.commands.WheelRadiusCommand;
import frc.robot.util.PoseEX;
import frc.robot.util.SendableConsumer;


public class RobotContainer {

  private SendableChooser<Command> autoChooser;
  
  private double MaxSpeed = TunerConstants.kSpeedAt12VoltsMps;
  private double MaxAngularRate = TunerConstants.MAX_ANGULAR_RATE;

  private double speedPercent = .6;

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final CommandXboxController driverController = new CommandXboxController(0);

  private final CustomController customController = new CustomController(1);
  
  private final SwerveDrive drivetrain = new SwerveDrive(); // My drivetrain

  private final Elevator elevator = new Elevator();

  private final Intake intake = new Intake();

  private final Arm arm = new Arm();
  
  private final Climber climber = new Climber();

  private final Ramp ramp = new Ramp();

  private final MainMechanism scoringMechanism = new MainMechanism(arm, intake, elevator, ramp);

  private final ClimbMechanism climbMechanism = new ClimbMechanism(arm, climber, ramp);
  // private final ObjectDetection objectDetection = new ObjectDetection("Test",
  //   new Transform3d(new Translation3d(0,-.101, .522), new Rotation3d(0, Units.degreesToRadians(-20), Units.degreesToRadians(0))), ()->drivetrain.getPose());

  private final FactoryCommands factoryCommands = new FactoryCommands(drivetrain, scoringMechanism);
  
  private final OptionController optionController = new OptionController(customController, factoryCommands, ()-> intake.hasCoral(), ()-> intake.hasAlgae());

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(0).withRotationalDeadband(0)
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); 

  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

  private final Telemetry logger = new Telemetry(MaxSpeed);

  private void configureBindings() {
    drivetrain.resetPose(new Pose2d(7,5,Rotation2d.fromDegrees(180)));
  
    drivetrain.setDefaultCommand(
        drivetrain.applyRequest(() -> drive.withVelocityX(-driverController.getLeftY() * speedPercent * MaxSpeed * MathUtil.clamp(1/(Math.abs(elevator.getPosition())),.1,1))
            .withVelocityY(-driverController.getLeftX() * speedPercent * MaxSpeed * MathUtil.clamp(1/(Math.abs(elevator.getPosition())),.1,1))
            .withRotationalRate(-driverController.getRightX()/*driverController.getRawAxis(2)*/ * .65 * MaxAngularRate * MathUtil.clamp(1/(Math.abs(elevator.getPosition())),.1,1))
    ));

    drivetrain.getDriveIO().registerTelemetry((log)->logger.telemeterize(log));
    
    /* Controller Bindings */
    // createManualControls();
    createPresetControls();
    // createAutoPathControls();

    // customController.rawButtonPressed(1).onTrue(Commands.print("1"));
    // customController.rawButtonPressed(2).onTrue(Commands.print("2"));
    // customController.rawButtonPressed(3).onTrue(Commands.print("3"));
    // customController.rawButtonPressed(4).onTrue(Commands.print("4"));
    // customController.rawButtonPressed(5).onTrue(Commands.print("5"));
    // customController.rawButtonPressed(6).onTrue(Commands.print("6"));
    // customController.rawButtonPressed(7).onTrue(Commands.print("7"));
    // customController.rawButtonPressed(8).onTrue(Commands.print("8"));
    // customController.rawButtonPressed(9).onTrue(Commands.print("9"));
    // customController.rawButtonPressed(10).onTrue(Commands.print("10"));
    
    driverController.leftStick().onTrue(Commands.runOnce(() -> drivetrain.seedFieldRelative(drivetrain.getPose().getRotation())));
    
    // driverController.rightTrigger(.5).whileTrue(drivetrain.applyRequest(() -> brake));
    // driverController.leftTrigger(.5).whileTrue(factoryCommands.autoIntakeCoral(0));

    //TODO was deffered, i switched to not, make sure it works in all scenarios
    new Trigger(()->DriverStation.isTeleop()).onTrue(Commands.runOnce(()->{
      // DynamicObstacle.setDynamicObstacles("avoidAlgae",drivetrain.getPose().getTranslation());
      // if (Robot.isSimulation()){
      //   drivetrain.seedFieldRelative(Rotation2d.fromDegrees(90));
      //   return;
      // }
      if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red){
        drivetrain.seedFieldRelative(Rotation2d.fromDegrees(180));
      }else{
        drivetrain.seedFieldRelative(Rotation2d.fromDegrees(0));
      }
    }));
  }

  public void createManualControls(){
    double[] elevatorAndArm = new double[]{0,0};
    elevator.setDefaultCommand(elevator.reachGoal(()->elevatorAndArm[0]));
    arm.setDefaultCommand(arm.reachGoal(()->elevatorAndArm[1]));
    driverController.leftTrigger(.5).whileTrue(
      Commands.run(()->{
        elevatorAndArm[0]+=.02;
        elevatorAndArm[0] = MathUtil.clamp(elevatorAndArm[0],0,2);
      }));
    driverController.rightTrigger(.5).whileTrue(
      Commands.run(()->{
        elevatorAndArm[0]-=.02;
        elevatorAndArm[0] = MathUtil.clamp(elevatorAndArm[0],0,2);
      }));
    driverController.leftBumper().whileTrue(
      Commands.run(()->{
        elevatorAndArm[1]-=.01;
        elevatorAndArm[1] = MathUtil.clamp(elevatorAndArm[1],0,Units.degreesToRotations(70));
      }));
    driverController.rightBumper().whileTrue(
      Commands.run(()->{
        elevatorAndArm[1]+=.01;
        elevatorAndArm[1] = MathUtil.clamp(elevatorAndArm[1],0,Units.degreesToRotations(70));
      }));
    driverController.a().whileTrue(intake.setVelocity(30));
    driverController.b().whileTrue(scoringMechanism.intake().alongWith(Commands.runOnce(()->{
      elevatorAndArm[0] = 0;
      elevatorAndArm[1] = 0;
    })));
  };

  public void createPresetControls(){
    driverController.povUp().onTrue(Commands.runOnce(()->optionController.setReefLevel(1)));
    driverController.povRight().onTrue(Commands.runOnce(()->optionController.setReefLevel(2)));
    driverController.povDown().onTrue(Commands.runOnce(()->optionController.setReefLevel(3)));
    driverController.povLeft().onTrue(Commands.runOnce(()->optionController.setReefLevel(4)));
    // driverController.povRight().whileTrue(scoringMechanism.preset(2));
    // driverController.povDown().whileTrue(scoringMechanism.preset(3));
    // driverController.povLeft().whileTrue(scoringMechanism.preset(4));
    driverController.rightTrigger(.2).whileTrue(optionController.getScoreLevel());
    driverController.y().whileTrue(elevator.reachGoal(0).alongWith(arm.reachGoal(0)));
    driverController.b().whileTrue(climbMechanism.prepare());
    driverController.x().whileTrue(climbMechanism.climb());
    // driverController.a().whileTrue(intake.setVelocity(30).alongWith(elevator.reachGoal(0).alongWith(arm.reachGoal(-.22)).alongWith(ramp.reachGoal(0))));
    driverController.rightBumper().whileTrue(intake.setVelocity(60));
    driverController.leftBumper().whileTrue(optionController.resetOrIntake());
    driverController.leftTrigger(.2).whileTrue(optionController.getAlgaeIntakeLevel().alongWith(intake.setVelocity(-60)));
    driverController.start().whileTrue(optionController.getAutoAlgae());
    driverController.rightStick().whileTrue(optionController.getAutoCoral());
    driverController.back().whileTrue(optionController.getAutoCoralPosition());

    customController.fixedButtonPressed(17).onTrue(Commands.runOnce(()->{speedPercent = .6;}));
    customController.fixedButtonPressed(18).onTrue(Commands.runOnce(()->{speedPercent = .8;}));

    customController.fixedButtonPressed(17).and(()->customController.getFixedButton(18)).whileTrue(intake.setVelocity(30).alongWith(elevator.reachGoal(0).alongWith(arm.reachGoal(-.22)).alongWith(ramp.reachGoal(0))));
  }

  public void createAutoPathControls(){
    //Auto Aligns, do everything but defence and player avoidance
    driverController.leftTrigger(.5).whileTrue(optionController.getAutoAlgae());
    driverController.rightTrigger(.5).whileTrue(optionController.getAutoCoral());

    //basic preset stuff if auto align is not working
    driverController.povUp().whileTrue(scoringMechanism.presetCoral(1));
    driverController.povRight().whileTrue(scoringMechanism.presetCoral(2));
    driverController.povDown().whileTrue(scoringMechanism.presetCoral(3));
    driverController.povLeft().whileTrue(scoringMechanism.presetCoral(4));

    driverController.leftBumper().whileTrue(intake.setVelocity(30));
    driverController.rightBumper().whileTrue(scoringMechanism.intake());
  }

  public RobotContainer() {
    drivetrain.configurePathPlanner();
    
    DataLogManager.start();
    DriverStation.startDataLog(DataLogManager.getLog());
    DataLogManager.logNetworkTables(true);
    
    configureAutonomousCommands();
    
    autoChooser = buildAutoChooser("", (data) -> data);

    // autoChooser.onChange((data)->{
    //   try{
    //     PathPlannerAuto auto = new PathPlannerAuto(data.getName());
    //     drivetrain.resetPose((!DriverStation.getAlliance().isEmpty() && DriverStation.getAlliance().get() == Alliance.Red)
    //       ? PoseEX.pose180(auto.getStartingPose())
    //       : auto.getStartingPose());
    //   } catch(Exception e){
    //     drivetrain.resetPose((!DriverStation.getAlliance().isEmpty() && DriverStation.getAlliance().get() == Alliance.Red)
    //       ? PoseEX.pose180(WaitAutos.getStartingPose(data.getName()))
    //       : WaitAutos.getStartingPose(data.getName()));
    //   }
    // });
    
    SmartDashboard.putData("Auto Chooser", autoChooser);

    SmartDashboard.putData(CommandScheduler.getInstance());

    configureBindings();

    //How you might make a choreo only path
    // autoChooser.addOption("ChoreoPath", ChoreoEX.getChoreoGroupPath(true,new String[]{"shootPreAmp","intake4","shoot4M","intake5","shoot5M","intake6","shoot6M","intake7","shoot7M"}));
    autoChooser.addOption("RightAuto",
      WaitAutos.createBranchCommand("RightAuto", new Pose2d(7.2,1.662,Rotation2d.fromDegrees(180)), "",
        BranchInstruction.of(BeginPose.BeginRight, ShootPose.PlaceE,4),
        BranchInstruction.of(IntakePose.FeederTwo, ShootPose.PlaceC,4),
        BranchInstruction.of(IntakePose.FeederTwo, ShootPose.PlaceD,4),
        BranchInstruction.of(IntakePose.FeederTwo, ShootPose.PlaceE,4)
    ));
    autoChooser.addOption("MiddleAuto",
      WaitAutos.createBranchCommand("MiddleAuto", new Pose2d(7.2,4.025,Rotation2d.fromDegrees(180)), "",
        BranchInstruction.of(BeginPose.BeginMiddle, ShootPose.PlaceH,4),
        BranchInstruction.of(IntakePose.FeederOne, ShootPose.PlaceK,4),
        BranchInstruction.of(IntakePose.FeederOne, ShootPose.PlaceL,4),
        BranchInstruction.of(IntakePose.FeederOne, ShootPose.PlaceA,4)
    ));
    autoChooser.addOption("LeftAuto",
      WaitAutos.createBranchCommand("LeftAuto", new Pose2d(7.2,6.338,Rotation2d.fromDegrees(180)), "",
        BranchInstruction.of(BeginPose.BeginLeft, ShootPose.PlaceJ,4),
        BranchInstruction.of(IntakePose.FeederOne, ShootPose.PlaceK,4),
        BranchInstruction.of(IntakePose.FeederOne, ShootPose.PlaceL,4),
        BranchInstruction.of(IntakePose.FeederOne, ShootPose.PlaceA,4)
    ));
    // autoChooser.addOption("WaitAuto",
    //   WaitAutos.createBranchCommand("WaitAuto", new Pose2d(7.578,1.662,Rotation2d.fromDegrees(180)), "",
    //     BranchInstruction.of(BeginPose.BeginRight, ShootPose.PlaceF,4),
    //     BranchInstruction.of(IntakePose.FeederTwo, ShootPose.PlaceA,4),
    //     BranchInstruction.of(IntakePose.FeederTwo, ShootPose.PlaceB,4),
    //     BranchInstruction.of(IntakePose.FeederTwo, ShootPose.PlaceC,4),
    //     BranchInstruction.of(IntakePose.FeederTwo, ShootPose.PlaceD,4),
    //     BranchInstruction.of(IntakePose.FeederTwo, ShootPose.PlaceE,4),
    //     BranchInstruction.of(IntakePose.FeederTwo, ShootPose.PlaceG,4),
    //     BranchInstruction.of(IntakePose.FeederOne, ShootPose.PlaceH,4),
    //     BranchInstruction.of(IntakePose.FeederOne, ShootPose.PlaceI,4),
    //     BranchInstruction.of(IntakePose.FeederOne, ShootPose.PlaceJ,4),
    //     BranchInstruction.of(IntakePose.FeederOne, ShootPose.PlaceK,4),
    //     BranchInstruction.of(IntakePose.FeederOne, ShootPose.PlaceL,4),
    //     BranchInstruction.of(IntakePose.FeederOne, ShootPose.PlaceA,3),
    //     BranchInstruction.of(IntakePose.FeederTwo, ShootPose.PlaceB,3),
    //     BranchInstruction.of(IntakePose.FeederTwo, ShootPose.PlaceC,3),
    //     BranchInstruction.of(IntakePose.FeederTwo, ShootPose.PlaceD,3),
    //     BranchInstruction.of(IntakePose.FeederTwo, ShootPose.PlaceE,3),
    //     BranchInstruction.of(IntakePose.FeederTwo, ShootPose.PlaceF,3),
    //     BranchInstruction.of(IntakePose.FeederTwo, ShootPose.PlaceG,3),
    //     BranchInstruction.of(IntakePose.FeederOne, ShootPose.PlaceH,3),
    //     BranchInstruction.of(IntakePose.FeederOne, ShootPose.PlaceI,3),
    //     BranchInstruction.of(IntakePose.FeederOne, ShootPose.PlaceJ,3),
    //     BranchInstruction.of(IntakePose.FeederOne, ShootPose.PlaceK,3),
    //     BranchInstruction.of(IntakePose.FeederOne, ShootPose.PlaceL,3),
    //     BranchInstruction.of(IntakePose.FeederOne, ShootPose.PlaceA,2),
    //     BranchInstruction.of(IntakePose.FeederTwo, ShootPose.PlaceB,2),
    //     BranchInstruction.of(IntakePose.FeederTwo, ShootPose.PlaceC,2),
    //     BranchInstruction.of(IntakePose.FeederTwo, ShootPose.PlaceD,2),
    //     BranchInstruction.of(IntakePose.FeederTwo, ShootPose.PlaceE,2),
    //     BranchInstruction.of(IntakePose.FeederTwo, ShootPose.PlaceF,2),
    //     BranchInstruction.of(IntakePose.FeederTwo, ShootPose.PlaceG,2),
    //     BranchInstruction.of(IntakePose.FeederOne, ShootPose.PlaceH,2),
    //     BranchInstruction.of(IntakePose.FeederOne, ShootPose.PlaceI,2),
    //     BranchInstruction.of(IntakePose.FeederOne, ShootPose.PlaceJ,2),
    //     BranchInstruction.of(IntakePose.FeederOne, ShootPose.PlaceK,2),
    //     BranchInstruction.of(IntakePose.FeederOne, ShootPose.PlaceL,2)
    //   ));
  }

  public void configureAutonomousCommands() {
    NamedCommands.registerCommand("intake", scoringMechanism.intake());
    NamedCommands.registerCommand("presetL4", scoringMechanism.presetCoral(4));
    NamedCommands.registerCommand("placeL4", scoringMechanism.scoreCoral(4));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  public static SendableChooser<Command> buildAutoChooser(
      String defaultAutoName,
      Function<Stream<PathPlannerAuto>, Stream<PathPlannerAuto>> optionsModifier) {
    if (!AutoBuilder.isConfigured()) {
      throw new RuntimeException(
          "AutoBuilder was not configured before attempting to build an auto chooser");
    }

    SendableChooser<Command> chooser = new SendableChooser<>();
    List<String> autoNames = AutoBuilder.getAllAutoNames();

    PathPlannerAuto defaultOption = null;
    List<PathPlannerAuto> options = new ArrayList<>();

    for (String autoName : autoNames) {
      PathPlannerAuto auto;
      try {
        auto = new PathPlannerAuto(autoName);
      } catch (Exception e) {
        auto = new PathPlannerAuto(Commands.none());
      }
      
      if (!defaultAutoName.isEmpty() && defaultAutoName.equals(autoName)) {
        defaultOption = auto;
      } else {
        options.add(auto);
      }
    }

    if (defaultOption == null) {
      chooser.setDefaultOption("None", Commands.none());
    } else {
      chooser.setDefaultOption(defaultOption.getName(), defaultOption);
      chooser.addOption("None", Commands.none());
    }

    optionsModifier
        .apply(options.stream())
        .forEach(auto -> chooser.addOption(auto.getName(), auto));

    return chooser;
  }
}

// public class RobotContainer {

//   private final Elevator elevator = new Elevator();

//   private final Intake intake = new Intake();

//   private final Arm arm = new Arm();

//   private final Climber climber = new Climber();

//   private final Ramp ramp = new Ramp();

//   private final CustomController controller = new CustomController(1);

//   private void configureBindings() {
    
//   }


//   public RobotContainer() {

//   }

//   public void configureAutonomousCommands() {

//   }

//   public Command getAutonomousCommand() {
//     return Commands.none();
//   }
// }

// public class RobotContainer {

//   private SendableChooser<Command> autoChooser;
  
//   private double MaxSpeed = TunerConstants.kSpeedAt12VoltsMps;
//   private double MaxAngularRate = TunerConstants.MAX_ANGULAR_RATE;

//   /* Setting up bindings for necessary control of the swerve drive platform */
//   private final CommandXboxController driverController = new CommandXboxController(0);

//   private final CustomController customController = new CustomController(1);
  
//   private final SwerveDrive drivetrain = new SwerveDrive(); // My drivetrain

//   private final Elevator elevator = new Elevator();

//   private final Intake intake = new Intake();

//   private final Arm arm = new Arm();
  
//   private final Climber climber = new Climber();

//   private final Ramp ramp = new Ramp();

//   private final MainMechanism scoringMechanism = new MainMechanism(arm, intake, elevator, ramp);

//   private final ClimbMechanism climbMechanism = new ClimbMechanism(climber, ramp);
//   // private final ObjectDetection objectDetection = new ObjectDetection("Test",
//   //   new Transform3d(new Translation3d(0,-.101, .522), new Rotation3d(0, Units.degreesToRadians(-20), Units.degreesToRadians(0))), ()->drivetrain.getPose());

//   private final FactoryCommands factoryCommands = new FactoryCommands(drivetrain, scoringMechanism);
  
//   private final OptionController optionController = new OptionController(customController, factoryCommands, ()-> intake.hasCoral(), ()-> intake.hasAlgae());

//   private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
//       .withDeadband(0).withRotationalDeadband(0)
//       .withDriveRequestType(DriveRequestType.OpenLoopVoltage); 

//   private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

//   private final Telemetry logger = new Telemetry(MaxSpeed);

//   private void configureBindings() {
//     drivetrain.resetPose(new Pose2d(7,5,new Rotation2d()));
  
//     drivetrain.setDefaultCommand(
//         drivetrain.applyRequest(() -> drive.withVelocityX(-driverController.getLeftY() * 1.00 * MaxSpeed)
//             .withVelocityY(-driverController.getLeftX() * 1.00 * MaxSpeed)
//             .withRotationalRate(-driverController.getRightX()/*driverController.getRawAxis(2)*/ * 2/3 * MaxAngularRate)
//     ));

//     drivetrain.getDriveIO().registerTelemetry((log)->logger.telemeterize(log));

//     driverController.y().onTrue(Commands.runOnce(() -> drivetrain.seedFieldRelative(drivetrain.getPose().getRotation())));

//     driverController.rightTrigger(.5).whileTrue(drivetrain.applyRequest(()->brake));

//     //TODO was deffered, i switched to not, make sure it works in all scenarios
//     new Trigger(()->DriverStation.isTeleop()).onTrue(Commands.runOnce(()->{
//       // DynamicObstacle.setDynamicObstacles("avoidAlgae",drivetrain.getPose().getTranslation());
//       if (Robot.isSimulation()){
//         drivetrain.seedFieldRelative(Rotation2d.fromDegrees(90));
//         return;
//       }
//       if (DriverStation.getAlliance().get() == Alliance.Red){
//         drivetrain.seedFieldRelative(Rotation2d.fromDegrees(180));
//       }else{
//         drivetrain.seedFieldRelative(Rotation2d.fromDegrees(0));
//       }
//     }));
//   }

//   public RobotContainer() {
//     drivetrain.configurePathPlanner();
    
//     DataLogManager.start();
//     DriverStation.startDataLog(DataLogManager.getLog());
//     DataLogManager.logNetworkTables(true);
    
//     configureAutonomousCommands();
    
//     autoChooser = buildAutoChooser("", (data) -> data);

//     // autoChooser.onChange((data)->{
//     //   try{
//     //     PathPlannerAuto auto = new PathPlannerAuto(data.getName());
//     //     drivetrain.resetPose((!DriverStation.getAlliance().isEmpty() && DriverStation.getAlliance().get() == Alliance.Red)
//     //       ? PoseEX.pose180(auto.getStartingPose())
//     //       : auto.getStartingPose());
//     //   } catch(Exception e){
//     //     drivetrain.resetPose((!DriverStation.getAlliance().isEmpty() && DriverStation.getAlliance().get() == Alliance.Red)
//     //       ? PoseEX.pose180(WaitAutos.getStartingPose(data.getName()))
//     //       : WaitAutos.getStartingPose(data.getName()));
//     //   }
//     // });
    
//     SmartDashboard.putData("Auto Chooser", autoChooser);

//     SmartDashboard.putData(CommandScheduler.getInstance());

//     configureBindings();

//     //How you might make a choreo only path
//     // autoChooser.addOption("ChoreoPath", ChoreoEX.getChoreoGroupPath(true,new String[]{"shootPreAmp","intake4","shoot4M","intake5","shoot5M","intake6","shoot6M","intake7","shoot7M"}));
    
//     autoChooser.addOption("WaitAuto",
//       WaitAutos.createBranchCommand("WaitAuto", new Pose2d(7.578,1.662,Rotation2d.fromDegrees(180)), "",
//         BranchInstruction.of(BeginPose.BeginRight, ShootPose.PlaceF,4),
//         BranchInstruction.of(IntakePose.FeederTwo, ShootPose.PlaceA,4),
//         BranchInstruction.of(IntakePose.FeederTwo, ShootPose.PlaceB,4),
//         BranchInstruction.of(IntakePose.FeederTwo, ShootPose.PlaceC,4),
//         BranchInstruction.of(IntakePose.FeederTwo, ShootPose.PlaceD,4),
//         BranchInstruction.of(IntakePose.FeederTwo, ShootPose.PlaceE,4),
//         BranchInstruction.of(IntakePose.FeederTwo, ShootPose.PlaceG,4),
//         BranchInstruction.of(IntakePose.FeederOne, ShootPose.PlaceH,4),
//         BranchInstruction.of(IntakePose.FeederOne, ShootPose.PlaceI,4),
//         BranchInstruction.of(IntakePose.FeederOne, ShootPose.PlaceJ,4),
//         BranchInstruction.of(IntakePose.FeederOne, ShootPose.PlaceK,4),
//         BranchInstruction.of(IntakePose.FeederOne, ShootPose.PlaceL,4)
//       ));

//       double[] vals = new double[]{0,0,0};
//       SendableConsumer.createSendableChooser("ScoringPositions"
//         ,new String[]{"ElevatorMeters", "ArmDegrees", "IntakeVelocity"}
//         ,(data)->{vals[0] = data[0];vals[1] = data[1];vals[2] = data[2];}
//         ,vals);
//       driverController.a().whileTrue(scoringMechanism.testPosition(()->vals[0],()->vals[1],()->vals[2]).withInterruptBehavior(InterruptionBehavior.kCancelSelf));
//   }

//   public void configureAutonomousCommands() {
//     NamedCommands.registerCommand("intake", scoringMechanism.intake());
//     NamedCommands.registerCommand("presetL4", scoringMechanism.presetL4());
//     NamedCommands.registerCommand("placeL4", scoringMechanism.scoreL4());
//   }

//   public Command getAutonomousCommand() {
//     return autoChooser.getSelected();
//   }

//   public static SendableChooser<Command> buildAutoChooser(
//       String defaultAutoName,
//       Function<Stream<PathPlannerAuto>, Stream<PathPlannerAuto>> optionsModifier) {
//     if (!AutoBuilder.isConfigured()) {
//       throw new RuntimeException(
//           "AutoBuilder was not configured before attempting to build an auto chooser");
//     }

//     SendableChooser<Command> chooser = new SendableChooser<>();
//     List<String> autoNames = AutoBuilder.getAllAutoNames();

//     PathPlannerAuto defaultOption = null;
//     List<PathPlannerAuto> options = new ArrayList<>();

//     for (String autoName : autoNames) {
//       PathPlannerAuto auto;
//       try {
//         auto = new PathPlannerAuto(autoName);
//       } catch (Exception e) {
//         auto = new PathPlannerAuto("");
//       }
      
//       if (!defaultAutoName.isEmpty() && defaultAutoName.equals(autoName)) {
//         defaultOption = auto;
//       } else {
//         options.add(auto);
//       }
//     }

//     if (defaultOption == null) {
//       chooser.setDefaultOption("None", Commands.none());
//     } else {
//       chooser.setDefaultOption(defaultOption.getName(), defaultOption);
//       chooser.addOption("None", Commands.none());
//     }

//     optionsModifier
//         .apply(options.stream())
//         .forEach(auto -> chooser.addOption(auto.getName(), auto));

//     return chooser;
//   }
// }