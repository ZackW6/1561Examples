package frc.robot.gameConnection;

import java.io.*;
import java.lang.reflect.Type;
import java.net.*;
import com.google.gson.Gson;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.gameConnection.Rectangle;
import frc.robot.subsystems.SimSwerve;

public class GameConnection {

    private static GameConnection instance;
    private static Pose2d robotPose = new Pose2d();
    private static ServerSocket serverSocket;

    private static SimSwerve m_drivetrain;

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

                        String request = in.readLine(); // Read Unity request

                        // CollisionData collisionData = new Gson().fromJson(request, CollisionData.class);
                        Vector2 collisionData = new Gson().fromJson(request, Vector2.class);
                        if (m_drivetrain != null && collisionData != null && collisionData.x != 0){
                            // m_drivetrain.handleSimCollision(collisionData.robot, collisionData.other);
                            m_drivetrain.handleSimCollision(collisionData);
                        }
                        
                        // Respond with some data
                        String response = new Gson().toJson(robotPose);
                        out.println(response);
                    }
                    clientSocket.close();
                    Thread.sleep((long) 20);
                }
            } catch (Exception e) {
                System.out.println("Main thread ended");
            }
        });
        comThread.start();
    }

    public static void assignDrivetrain(SimSwerve drivetrain){
        m_drivetrain = drivetrain;
    }

    public static void initConnection(){
        if (instance == null){
            instance = new GameConnection();
        }
    }

    public static void setRobotPose(Pose2d pose){
        robotPose = pose;
    }

    public class CollisionData{
        public Rectangle robot;
        public Rectangle other;
        public CollisionData(Rectangle robot, Rectangle other){
            this.robot = robot;
            this.other = other;
        }
    }
}