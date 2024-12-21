package frc.robot.gameConnection;

import java.io.*;
import java.lang.reflect.Type;
import java.net.*;
import com.google.gson.Gson;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class GameConnection {

    private static GameConnection instance;
    private static Pose2d robotPose;
    private static ServerSocket serverSocket;

    private static CommandSwerveDrivetrain m_drivetrain;

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
                        CollisionVector collisionVector = new Gson().fromJson(request, CollisionVector.class);
                        if (m_drivetrain != null && collisionVector.m_xN+collisionVector.m_yN != 0 ){
                            m_drivetrain.handleSimCollision(new Vector2(collisionVector.m_xN, collisionVector.m_yN)
                            , new Vector2(collisionVector.m_yVel, collisionVector.m_yVel), collisionVector.m_rVel);
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

    public static void assignDrivetrain(CommandSwerveDrivetrain drivetrain){
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

    public class CollisionVector{
        public double m_xN;
        public double m_yN;
        public double m_xVel;
        public double m_yVel;
        public double m_rVel;
        public CollisionVector(float xN, float yN, float xVel, float yVel, float rVel){
            m_xN = xN;
            m_yN = yN;

            m_xVel = xVel;
            m_yVel = yVel;
            m_rVel = rVel;
        }
    }

    public static class Vector2{

        public double x;
        public double y;

        public Vector2(double x, double y){
            this.x = x;
            this.y = y;
        }

        public static Vector2 Reflect(Vector2 inDirection, Vector2 inNormal)
        {
            double num = -2 * Dot(inNormal, inDirection);
            return new Vector2(num * inNormal.x + inDirection.x, num * inNormal.y + inDirection.y);
        }

        public static double Dot(Vector2 lhs, Vector2 rhs){
            return lhs.x * rhs.x + lhs.y * rhs.y;
        }

    }
}