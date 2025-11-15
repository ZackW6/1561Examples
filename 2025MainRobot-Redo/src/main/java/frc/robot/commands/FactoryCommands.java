// package frc.robot.commands;

// import java.util.Optional;
// import java.util.Set;
// import java.util.function.BooleanSupplier;

// import com.ctre.phoenix6.swerve.SwerveRequest;
// import com.ctre.phoenix6.swerve.SwerveRequest.ApplyFieldSpeeds;
// import com.ctre.phoenix6.swerve.SwerveRequest.ApplyRobotSpeeds;
// import com.pathplanner.lib.auto.CommandUtil;
// import com.pathplanner.lib.commands.PathPlannerAuto;

// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Transform2d;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.math.kinematics.ChassisSpeeds;
// import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj.DriverStation.Alliance;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.Commands;
// import edu.wpi.first.wpilibj2.command.Subsystem;
// import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
// import frc.robot.commands.PathOnTheFly.PathConfig;
// import frc.robot.constants.GameData;
// import frc.robot.generated.TunerConstants;
// import frc.robot.subsystems.MainMechanism;
// import frc.robot.subsystems.MainMechanism.IntakeSpeeds;
// import frc.robot.subsystems.MainMechanism.Positions;
// import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;
// import frc.robot.subsystems.swerve.SwerveDrive;
// import frc.robot.subsystems.swerve.swerveHelpers.ShareDrive;
// import frc.robot.subsystems.swerve.swerveHelpers.Vector2;
// import frc.robot.util.PoseEX;

// public class FactoryCommands {

//     public static final double positionalToleranceMeters = .05;
//     public static final double rotationalToleranceRotations = .1;

//     //TODO if auto breaks, could be here
//     public static final double maxSpeedAutoAlign = 1.2;//1.6
//     public static final double maxSpeedAutoCoral = .5;//1.6
//     public static final double maxSpeedAutoIntake = 1.3;
//     public static final double lowerElevatorDist = 1.4;
//     public static final double raiseElevatorDist = 2.35;

//     private final PIDController speedsPID = new PIDController(6, 0, 0);
//     private final PIDController rotationPID = new PIDController(6, 0, 0);

//     public final SwerveDrive drivetrain;

//     public final CommandXboxController xboxController;

//     public final MainMechanism scoringMechanism;

//     public final Set<Subsystem> mainSubsytems;
//     public final Set<Subsystem> scoringSubsytems;
//     public final Set<Subsystem> presetSubsytems;

//     private static FactoryCommands instance;
    
//     private final ApplyFieldSpeeds fieldSpeedRequest = new ApplyFieldSpeeds();

//     private final ApplyRobotSpeeds robotSpeedRequest = new ApplyRobotSpeeds();


//     public FactoryCommands(SwerveDrive drivetrain, CommandXboxController controller, MainMechanism scoringMechanism){
//         if (instance == null){
//             instance = this;
//         }
//         this.drivetrain = drivetrain;
//         this.xboxController = controller;
//         this.scoringMechanism = scoringMechanism;
//         mainSubsytems = Set.of(drivetrain, scoringMechanism.arm, scoringMechanism.elevator, scoringMechanism.intake);
//         scoringSubsytems = Set.of(scoringMechanism.arm, scoringMechanism.elevator, scoringMechanism.intake);
//         presetSubsytems = Set.of(scoringMechanism.arm, scoringMechanism.elevator);
//     }

//     public static Optional<FactoryCommands> getInstance(){
//         if (instance == null){
//             return Optional.empty();
//         }
//         return Optional.of(instance);
//     }

//     public Command autoToCoral(int place){
//         int clampedNum = Math.max(Math.min(place,12),1);
//         return Commands.defer(()->drivetrain.toPose(GameData.coralPose(clampedNum, DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red)
//         ,1.2,maxSpeedAutoAlign),Set.of());
//     }

//     public Command autoToAlgae(int place){
//         int clampedNum = Math.max(Math.min(place,6),1);
//         return Commands.defer(()->drivetrain.toPose(GameData.algaePose(clampedNum, DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red)
//         ,1.2,maxSpeedAutoAlign)
//             .until(()->drivetrain.getPose().minus(GameData.algaePose(clampedNum, DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red)).getTranslation().getNorm() < .05),Set.of())
//             .andThen(drivetrain.applyRequest(()->new SwerveRequest.RobotCentric().withVelocityX(.25)));
//     }

//     public Command autoToFeeder(int place, double rightOffset){
//         int clampedNum = Math.max(Math.min(place,2),1);
//         return Commands.defer(()->drivetrain.toPose(GameData.feederPose(clampedNum, DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red)
//         .plus(new Transform2d(0,rightOffset, new Rotation2d()))
//         ,3,4),Set.of());
//     }

//     public Command autoToFeeder(int place){
//         return autoToFeeder(place,0);
//     }

//     public Command autoToProcessor(){
//         return Commands.defer(()->(drivetrain.applyRequest(()->new SwerveRequest.RobotCentric().withVelocityX(-1)).alongWith(scoringMechanism.voltZero())).withTimeout(.6).andThen(drivetrain.toPose(GameData.processorPose(DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red)
//         ,2,maxSpeedAutoAlign)),Set.of());
//     }

//     public Command autoToNet(){
//         return drivetrain.applyRequest(()->new SwerveRequest.RobotCentric().withVelocityX(-1.25)).withTimeout(.6).andThen(Commands.defer(()->drivetrain.toPose(
//             PoseEX.closestTo(drivetrain.getPose(),
//             GameData.netPose(1,DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red),
//             GameData.netPose(2,DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red))
//         ,2,maxSpeedAutoAlign),Set.of()));
//     }

//     public Command autoScoreCoral(int place, int level){
//         return Commands.race(autoToCoral(place)
//         ,Commands.deadline(Commands.waitUntil(()->{
//             Pose2d drivetrainPose = drivetrain.getPose();
//             Pose2d coralPose = GameData.coralPose(place, DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red);
//             Transform2d comparingTransform = coralPose.minus(drivetrainPose);

//             return (comparingTransform.getTranslation().getNorm() < positionalToleranceMeters) 
//                 && (coralPose.getRotation().getRotations() - drivetrainPose.getRotation().getRotations() < rotationalToleranceRotations)
//                 && drivetrain.getDriveIO().getSpeeds().vyMetersPerSecond < .1;
//         })
//         ,scoringMechanism.presetCoral(level,()->{
//             Pose2d drivetrainPose = drivetrain.getPose();
//             Pose2d coralPose = GameData.coralPose(place, DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red);
//             Transform2d comparingTransform = coralPose.minus(drivetrainPose);

//             return comparingTransform.getTranslation().getNorm() < raiseElevatorDist ? 1/(Math.min(comparingTransform.getTranslation().getNorm(),5)/5): 0;
//         })).andThen(scoringMechanism.scoreCoral(level)));
//     }

//     public Command autoScoreNet(){
//         return Commands.race(autoToNet()
//         ,Commands.deadline(Commands.waitUntil(()->{
//             Pose2d drivetrainPose = drivetrain.getPose();
//             Pose2d algaePose = PoseEX.closestTo(drivetrain.getPose(),
//                 GameData.netPose(1,DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red),
//                 GameData.netPose(2,DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red));
//             Transform2d comparingTransform = algaePose.minus(drivetrainPose);

//             return (comparingTransform.getTranslation().getNorm() < positionalToleranceMeters) 
//                 && (algaePose.getRotation().getRotations() - drivetrainPose.getRotation().getRotations() < rotationalToleranceRotations);
//         })
//         ,scoringMechanism.preset(Positions.AlgaeN,()->{
//             Pose2d drivetrainPose = drivetrain.getPose();
//             Pose2d algaePose = PoseEX.closestTo(drivetrain.getPose(),
//                 GameData.netPose(1,DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red),
//                 GameData.netPose(2,DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red));
//             Transform2d comparingTransform = algaePose.minus(drivetrainPose);

//             return comparingTransform.getTranslation().getNorm() < raiseElevatorDist ? 1/(Math.min(comparingTransform.getTranslation().getNorm(),5)/5): 0;
//         })).andThen(scoringMechanism.score(Positions.AlgaeN, IntakeSpeeds.ShootAlgae)));
//     }

//     public Command autoScoreProcessor(){
//         return Commands.race(autoToProcessor()
//         ,Commands.deadline(Commands.waitUntil(()->{
//             Pose2d drivetrainPose = drivetrain.getPose();
//             Pose2d processorPose = GameData.processorPose(DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red);
//             Transform2d comparingTransform = processorPose.minus(drivetrainPose);

//             return (comparingTransform.getTranslation().getNorm() < positionalToleranceMeters) 
//                 && (processorPose.getRotation().getRotations() - drivetrainPose.getRotation().getRotations() < rotationalToleranceRotations);
//         })
//         ,scoringMechanism.preset(Positions.AlgaeP)
//         ,scoringMechanism.intake.setVelocity(IntakeSpeeds.HoldAlgae.getVelocity())).andThen(scoringMechanism.score(Positions.AlgaeP, IntakeSpeeds.ShootAlgae)));
//     }

//     public Command autoScoreAlgae(int level){
//         return Commands.either(autoScoreProcessor(), autoScoreNet(), ()->level == 1);
//     }

//     public Command autoIntakeCoral(int place){
//         return autoIntakeCoral(place, 0);
//     }

//     public Command autoIntakeCoral(int place, double rightOffset){
//         return Commands.race(autoToFeeder(place, rightOffset)
//             , Commands.waitUntil(()->PoseEX.getDistanceFromPoseMeters(drivetrain.getPose()
//                 , GameData.reefCenterPose(DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red)) > lowerElevatorDist)
//         .andThen(scoringMechanism.intake()));
//     }

//     public Command autoIntakeAlgae(int place){
//         return Commands.race(autoToAlgae(place), scoringMechanism.voltZero().withTimeout(.5).andThen(scoringMechanism.intakeAlgae((place%2)+1)));
//     }

//     public Command teenyPush(){
//         return drivetrain.applyRequest(()->new SwerveRequest.RobotCentric().withVelocityX(-2)).withTimeout(.2);
//     }

//     public Command backupIntakebackupAlgae(int place){
//         return drivetrain.applyRequest(()->new SwerveRequest.RobotCentric().withVelocityX(-2)).withTimeout(.4)
//         .andThen(autoIntakeAlgae(place))
//         .andThen(drivetrain.applyRequest(()->new SwerveRequest.RobotCentric().withVelocityX(-2)).withTimeout(.4));
//     }
// }