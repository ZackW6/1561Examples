package frc.robot.util;

import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotation;

import java.lang.reflect.Field;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.dyn4j.geometry.Rectangle;
import org.dyn4j.geometry.Vector2;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.IntakeSimulation.IntakeSide;
import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;
import org.ironmaple.simulation.seasonspecific.crescendo2024.CrescendoNoteOnField;
import org.ironmaple.simulation.seasonspecific.crescendo2024.NoteOnFly;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeAlgaeOnField;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeAlgaeOnFly;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnFly;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.GameData;

import frc.robot.Robot;

public class MapleSimWorld{

    private static final MapleSimWorld instance;
    static{
        if (Robot.isSimulation()){
            instance = new MapleSimWorld();
        }else{
            instance = null;
        }
    }

    private static class IntakeInfo{
        public final ArrayList<BooleanSupplier> activationRequirements;
        public Consumer<Boolean> hasPiece;
        public final IntakeSimulation intakeSimulation;
        public final String key;

        public IntakeInfo(IntakeSimulation intakeSimulation, String key){
            this.intakeSimulation = intakeSimulation;
            intakeSimulation.register();
            try {
                Field type;
                type = IntakeSimulation.class.getDeclaredField("targetedGamePieceType");
                type.setAccessible(true);
                if (type.get(intakeSimulation).equals("Coral")){
                    Field gamePieceAmount;
                    gamePieceAmount = IntakeSimulation.class.getDeclaredField("gamePiecesInIntakeCount");
                    gamePieceAmount.setAccessible(true);
                    gamePieceAmount.set(intakeSimulation, 1);
                    System.out.println(intakeSimulation.getGamePiecesAmount());
                }
            } catch (Exception e){

            }
             // Bypass private access
            activationRequirements = new ArrayList<>();
            this.key = key;
        }

        public void addRequirements(BooleanSupplier requirement){
            activationRequirements.add(requirement);
        }
    }

    private static class ShootInfo{
        public final ArrayList<BooleanSupplier> activationRequirements;
        public final Supplier<Transform3d> shootVector;
        public final DoubleSupplier shootSpeed;
        public final String coralOrAlgae;
        public Supplier<Pose2d> robotPose = null;

        public ShootInfo(Supplier<Transform3d> vectorOfShoot, DoubleSupplier shootSpeed, String coralOrAlgae){
            activationRequirements = new ArrayList<>();
            shootVector = vectorOfShoot;
            this.shootSpeed = shootSpeed;
            this.coralOrAlgae = coralOrAlgae;
        }

        public ShootInfo(Supplier<Pose2d> robotPose, Supplier<Transform3d> vectorOfShoot, DoubleSupplier shootSpeed, String coralOrAlgae){
            activationRequirements = new ArrayList<>();
            shootVector = vectorOfShoot;
            this.shootSpeed = shootSpeed;
            this.coralOrAlgae = coralOrAlgae;
            this.robotPose = robotPose;
        }

        public void addRequirements(BooleanSupplier requirement){
            activationRequirements.add(requirement);
        }
    }

    private static HashMap<String, IntakeInfo> intakeKeys;
    private static HashMap<String, ShootInfo> shootKeys;

    private final NetworkTable networkTable;

    private final StructArrayPublisher<Pose3d> coralPublisher;

    /**
     * object detection sim relies on this as a subscriber
     */
    private final StructArrayPublisher<Pose3d> algaePublisher;

    private static AbstractDriveTrainSimulation drive;

    private final Thread updateThread;
    private MapleSimWorld(){
        intakeKeys = new HashMap<>();
        shootKeys = new HashMap<>();
        networkTable = NetworkTableInstance.getDefault().getTable("RealData");
        
        coralPublisher = networkTable
            .getStructArrayTopic("AllCoral", Pose3d.struct).publish();

        algaePublisher = networkTable
            .getStructArrayTopic("AllAlgae", Pose3d.struct).publish();
        
        updateThread = new Thread(()->{
            while(true){
                try {
                    double startTime = Timer.getFPGATimestamp();
                    SimulatedArena.getInstance().simulationPeriodic();
                    coralPublisher.set(SimulatedArena.getInstance().getGamePiecesArrayByType("Coral"));
                    algaePublisher.set(SimulatedArena.getInstance().getGamePiecesArrayByType("Algae"));

                    for (IntakeInfo info : intakeKeys.values()){
                        boolean intaking = true;
                        for (BooleanSupplier bool : info.activationRequirements){
                            if (!bool.getAsBoolean()){
                                intaking = false;
                            }
                        }
                        if (info.intakeSimulation.getGamePiecesAmount() != 0){
                            intaking = false;
                        }
                        if (intaking){
                            info.intakeSimulation.startIntake();
                        }else{
                            info.intakeSimulation.stopIntake();
                        }
                        if (info.intakeSimulation.getGamePiecesAmount() != 0 && shootKeys.get(info.key) != null){
                            boolean shooting = true;
                            for (BooleanSupplier bool : shootKeys.get(info.key).activationRequirements){
                                if (!bool.getAsBoolean()){
                                    shooting = false;
                                }
                            }
                            if (shooting){
                                info.intakeSimulation.obtainGamePieceFromIntake();
                                if (shootKeys.get(info.key).coralOrAlgae.equals("Coral")){
                                    if (shootKeys.get(info.key).robotPose != null){
                                        createShootingCoral(shootKeys.get(info.key).robotPose.get(),shootKeys.get(info.key).shootVector.get(), shootKeys.get(info.key).shootSpeed.getAsDouble());
                                    }else{
                                        createShootingCoral(shootKeys.get(info.key).shootVector.get(), shootKeys.get(info.key).shootSpeed.getAsDouble());
                                    }
                                    
                                }else{
                                    if (shootKeys.get(info.key).robotPose != null){
                                        createShootingAlgae(shootKeys.get(info.key).robotPose.get(), shootKeys.get(info.key).shootVector.get(), shootKeys.get(info.key).shootSpeed.getAsDouble());
                                    }else{
                                        createShootingAlgae(shootKeys.get(info.key).shootVector.get(), shootKeys.get(info.key).shootSpeed.getAsDouble());
                                    }
                                }
                                
                            }
                        }
                        if (info.hasPiece != null){
                            info.hasPiece.accept(info.intakeSimulation.getGamePiecesAmount() != 0);
                        }
                    }
                    Thread.sleep(20-(long)(Timer.getFPGATimestamp() - startTime)*1000);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
        });
        updateThread.setDaemon(true);
        updateThread.start();
    }

    public static Optional<MapleSimWorld> getInstance(){
        return Optional.ofNullable(instance);
    }

    public static void addMainDrive(AbstractDriveTrainSimulation mainDrive){
        drive = mainDrive;

        new Trigger(()->drive.getSimulatedDriveTrainPose().minus(GameData.feederPose(1,false)).getTranslation().getNorm() < 2)
        .whileTrue(Commands.sequence(Commands.waitSeconds(1),Commands.runOnce(()->{
            MapleSimWorld.createShootingCoral(GameData.feederPose(1,false)
                ,new Transform3d(0,0,1,new Rotation3d(0,0,0)), 0);
        }),Commands.waitSeconds(3)).repeatedly());

        new Trigger(()->drive.getSimulatedDriveTrainPose().minus(GameData.feederPose(2,false)).getTranslation().getNorm() < 2)
        .whileTrue(Commands.sequence(Commands.waitSeconds(1),Commands.runOnce(()->{
            MapleSimWorld.createShootingCoral(GameData.feederPose(2,false)
                ,new Transform3d(0,0,1,new Rotation3d(0,0,0)), 0);
        }),Commands.waitSeconds(3)).repeatedly());

        new Trigger(()->drive.getSimulatedDriveTrainPose().minus(GameData.feederPose(1,true)).getTranslation().getNorm() < 2)
        .whileTrue(Commands.sequence(Commands.waitSeconds(1),Commands.runOnce(()->{
            MapleSimWorld.createShootingCoral(GameData.feederPose(1,true)
                ,new Transform3d(0,0,1,new Rotation3d(0,0,0)), 0);
        }),Commands.waitSeconds(3)).repeatedly());

        new Trigger(()->drive.getSimulatedDriveTrainPose().minus(GameData.feederPose(2,true)).getTranslation().getNorm() < 2)
        .whileTrue(Commands.sequence(Commands.waitSeconds(1),Commands.runOnce(()->{
            MapleSimWorld.createShootingCoral(GameData.feederPose(2,true)
                ,new Transform3d(0,0,1,new Rotation3d(0,0,0)), 0);
        }),Commands.waitSeconds(3)).repeatedly());
    }

    public static void addIntakeSimulation(String uniqueIntakeKey, String target, IntakeSide side, double widthMeters, double lengthMeters, Translation2d position){
        intakeKeys.put(uniqueIntakeKey, new IntakeInfo(
            new IntakeSimulation(
                target,
                drive,
                getIntakeRectangle(lengthMeters, widthMeters, new Vector2(position.getX(), position.getY())),
                1)
            ,uniqueIntakeKey));
    }

    private static Rectangle getIntakeRectangle(
            double width, double lengthExtended, Vector2 translation) {
        final Rectangle intakeRectangle = new Rectangle(width, lengthExtended);

        //from center of robot
        intakeRectangle.translate(translation);

        return intakeRectangle;
    }
    public static void addIntakeRequirements(String key, BooleanSupplier requirement){
        IntakeInfo info = intakeKeys.get(key);
        if (info != null){
            info.addRequirements(requirement);
        }
    }

    public static void hasPiece(String key, Consumer<Boolean> consumer){
        IntakeInfo info = intakeKeys.get(key);
        if (info != null){
            info.hasPiece = consumer;
        }
    }

    public static void createShootingAlgae(Transform3d shooterPosition, double calculatedVelocity){
        createShootingAlgae(drive.getSimulatedDriveTrainPose(), shooterPosition, calculatedVelocity);
    }

    public static void createShootingAlgae(Pose2d from, Transform3d shooterPosition, double calculatedVelocity){

        Vector2 robotSpeeds = new Vector2();
        double angularVelocity = 0;
        if (drive != null){
            robotSpeeds = drive.getLinearVelocity();
            angularVelocity = drive.getAngularVelocity();
        }

        ReefscapeAlgaeOnFly algaeOnFly = new ReefscapeAlgaeOnFly(
            // Specify the position of the chassis when the algae is launched
            from.getTranslation(),
            // Specify the translation of the shooter from the robot center (in the shooter’s reference frame)
            shooterPosition.getTranslation().toTranslation2d(),
            // Specify the field-relative speed of the chassis, adding it to the initial velocity of the projectile
            new ChassisSpeeds(robotSpeeds.x, robotSpeeds.y, angularVelocity),
            // The shooter facing direction is the same as the robot’s facing direction
            Rotation2d.fromRadians(from.getRotation().getRadians() + shooterPosition.getRotation().toRotation2d().getRadians()),
                    // Add the shooter’s rotation,
            // Initial height of the flying note
            Meters.of(shooterPosition.getZ()),
            // The launch speed is proportional to the RPM; assumed to be 16 meters/second at 6000 RPM
            MetersPerSecond.of(calculatedVelocity),
            // The angle at which the note is launched
            Radians.of(shooterPosition.getRotation().getY())
        );
        algaeOnFly.enableBecomesGamePieceOnFieldAfterTouchGround();
        SimulatedArena.getInstance().addGamePieceProjectile(algaeOnFly);
    }

    public static void createShootingCoral(Transform3d shooterPosition, double calculatedVelocity){
        createShootingCoral(drive.getSimulatedDriveTrainPose(), shooterPosition, calculatedVelocity);
    }

    public static void createShootingCoral(Pose2d from, Transform3d shooterPosition, double calculatedVelocity){
        Vector2 robotSpeeds = drive.getLinearVelocity();
        ReefscapeCoralOnFly algaeOnFly = new ReefscapeCoralOnFly(
            // Specify the position of the chassis when the algae is launched
            from.getTranslation(),
            // Specify the translation of the shooter from the robot center (in the shooter’s reference frame)
            shooterPosition.getTranslation().toTranslation2d(),
            // Specify the field-relative speed of the chassis, adding it to the initial velocity of the projectile
            new ChassisSpeeds(robotSpeeds.x, robotSpeeds.y, drive.getAngularVelocity()),
            // The shooter facing direction is the same as the robot’s facing direction
            Rotation2d.fromRadians(from.getRotation().getRadians() + shooterPosition.getRotation().toRotation2d().getRadians()),
                    // Add the shooter’s rotation,
            // Initial height of the flying note
            Meters.of(shooterPosition.getZ()),
            // The launch speed is proportional to the RPM; assumed to be 16 meters/second at 6000 RPM
            MetersPerSecond.of(calculatedVelocity),
            // The angle at which the note is launched
            Radians.of(shooterPosition.getRotation().getY())
        );
        algaeOnFly.enableBecomesGamePieceOnFieldAfterTouchGround();
        SimulatedArena.getInstance().addGamePieceProjectile(algaeOnFly);
    }

    public static void addShooterSimulation(Supplier<Transform3d> shooterPosition, DoubleSupplier shootSpeed, String coralOrAlgae, String corrospondingIntakeKey){
        shootKeys.put(corrospondingIntakeKey, new ShootInfo(shooterPosition, shootSpeed, coralOrAlgae));
    }

    public static void addShooterSimulation(Supplier<Pose2d> robotPose, Supplier<Transform3d> shooterPosition, DoubleSupplier shootSpeed, String coralOrAlgae, String corrospondingIntakeKey){
        shootKeys.put(corrospondingIntakeKey, new ShootInfo(robotPose, shooterPosition, shootSpeed, coralOrAlgae));
    }

    public static void addShootRequirements(String corrospondingIntakeKey, BooleanSupplier requirements){
        shootKeys.get(corrospondingIntakeKey).addRequirements(requirements);
    }
}
