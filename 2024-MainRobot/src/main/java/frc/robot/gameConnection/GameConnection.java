package frc.robot.gameConnection;

import java.io.*;
import java.lang.reflect.Type;
import java.net.*;
import java.util.function.Consumer;
import java.util.function.Supplier;

import com.google.gson.Gson;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class GameConnection {

    private static GameConnection instance;
    private static GameInput m_speeds = new GameInput(0,0,0);
    private static ServerSocket serverSocket;
    private static GameData gameData = new GameData(new Pose2d());


    private static SwerveDriveKinematics driveKinematics;
    private static SwerveDrivePoseEstimator driveOdometry;
    private static Consumer<Rotation2d> gyroC;
    private static Supplier<ChassisSpeeds> m_chassisSpeeds;
    private static ChassisSpeeds lastSpeeds = new ChassisSpeeds();
    private static Supplier<SwerveModulePosition[]> modulePositions;

    private static boolean connected = false;

    private GameConnection(){
        try {
            serverSocket = new ServerSocket(7716);
        } catch (Exception e) {
            // TODO: handle exception
        }
        Thread comThread = new Thread(()->{
            try {
                while (true) {
                    
                    Socket clientSocket = serverSocket.accept();

                    try (BufferedReader in = new BufferedReader(new InputStreamReader(clientSocket.getInputStream()));
                        PrintWriter out = new PrintWriter(clientSocket.getOutputStream(), true)) {
                        connected = true;

                        String request = in.readLine(); // Read Unity request

                        gameData = new Gson().fromJson(request, GameData.class);
                        
                        // Respond with some data
                        String response = new Gson().toJson(m_speeds);
                        out.println(response);
                    } catch (Exception e) {
                        connected = false;
                    }
                    clientSocket.close();
                    Thread.sleep((long) 2);
                }
            } catch (Exception e) {
                connected = false;
                System.out.println("Main thread ended");
            }
        });
        comThread.start();
    }


    public static void initConnection(SwerveDriveKinematics kinematics,
            SwerveDrivePoseEstimator odometry,
            Consumer<Rotation2d> gyroConsumer,
            Supplier<ChassisSpeeds> chassisSpeeds,
            Supplier<SwerveModulePosition[]> modulePositionSupplier){
        driveKinematics = kinematics;
        driveOdometry = odometry;
        gyroC = gyroConsumer;
        m_chassisSpeeds = chassisSpeeds;
        modulePositions = modulePositionSupplier;
        if (instance == null){
            instance = new GameConnection();
        }
    }

    public static void setRobotSpeeds(GameInput speeds){
        m_speeds = speeds;
    }

    public static GameData getGameData(){
        return gameData;
    }

    public static void drivePeriodic(){
        try {
            if (driveKinematics == null || m_chassisSpeeds == null || !isConnected()){
                return;
            }
            if (lastSpeeds.vxMetersPerSecond == m_chassisSpeeds.get().vxMetersPerSecond){
                System.out.println(lastSpeeds.vxMetersPerSecond);
            }
            
            ChassisSpeeds speeds = m_chassisSpeeds.get();
            
            double[] newSpeeds = feildFromRobotRelativeSpeeds(speeds, getGameData().pose.getRotation());

            m_speeds.speeds.x = newSpeeds[0];
            m_speeds.speeds.y = newSpeeds[1];
            m_speeds.rotationVelocity = newSpeeds[2];
            if (gameData.seedTime != m_speeds.time && m_speeds.setPose != null){
                driveOdometry.resetPosition(Rotation2d.fromRadians(m_speeds.setPose.getRotation().getRadians()),
                    modulePositions.get(),
                    m_speeds.setPose);
                gyroC.accept(Rotation2d.fromRadians(m_speeds.setPose.getRotation().getRadians()));
                return;
            }
            
            gyroC.accept(Rotation2d.fromRadians(Math.PI-getGameData().pose.getRotation().getRadians()));
            Pose2d setPose = new Pose2d(getGameData().pose.getX(), getGameData().pose.getY()
                , Rotation2d.fromRadians(Math.PI-getGameData().pose.getRotation().getRadians()));
            driveOdometry.resetPosition(Rotation2d.fromRadians(Math.PI-getGameData().pose.getRotation().getRadians()),
                modulePositions.get(),
                setPose);
        } catch (Exception e) {
            connected = false;
        }
        
    }

    private static double[] feildFromRobotRelativeSpeeds(
        ChassisSpeeds speeds,
        Rotation2d currentAngle) {
        // CW rotation into chassis frame
        var rotated =
            new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond).rotateBy(currentAngle.unaryMinus().plus(Rotation2d.fromDegrees(180)));
        return new double[]{rotated.getX(), rotated.getY(), speeds.omegaRadiansPerSecond};
    }

    public static class GameData{
        public Pose2d pose;
        public double seedTime = 0;

        public GameData(Pose2d pose){
            this.pose = pose;
        }

        public GameData(){
            pose = new Pose2d();
        }
    }

    public static class GameInput{
        public Vector2 speeds;
        public double rotationVelocity;

        public Pose2d setPose = null;
        public double time = 0;

        public GameInput(double x, double y, double r){
            speeds = new Vector2(x, y);
            rotationVelocity = r;
        }

        public GameInput(Vector2 velocity, double r){
            speeds = velocity;
            rotationVelocity = r;
        }

        public void seedFieldRelative(Pose2d pose){
            setPose = pose;
            time = Timer.getFPGATimestamp();
        }
    }

    public static void seedFieldRelative(Pose2d pose){
        
        // Pose2d setPose = new Pose2d(pose.getX(), getGameData().pose.getY()
        //     , Rotation2d.fromRadians(Math.PI-pose.getRotation().getRadians()));

        driveOdometry.resetPosition(Rotation2d.fromRadians(pose.getRotation().getRadians()),
            modulePositions.get(),
            pose);
        gyroC.accept(Rotation2d.fromRadians(pose.getRotation().getRadians()));
        

        m_speeds.seedFieldRelative(pose);
        m_speeds.time = Timer.getFPGATimestamp();
        
    }

    public static boolean isConnected(){
        return connected;
    }
}