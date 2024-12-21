package frc.robot.gameConnection;

import edu.wpi.first.math.geometry.Pose2d;

public class Rectangle {
    Vector2 position;
    Vector2 velocity;
    double angle; // Rotation in radians
    double angularVelocity;
    double mass;
    double momentOfInertia;

    public Rectangle(Vector2 position, Vector2 velocity, double angle, double angularVelocity, double mass, double width, double height) {
        this.position = position;
        this.velocity = velocity;
        this.angle = angle;
        this.angularVelocity = angularVelocity;
        this.mass = mass;
        this.momentOfInertia = (mass * (width * width + height * height)) / 12.0; // Moment of inertia for a rectangle
    }

    public Rectangle(){
        this.position = new Vector2(0,0);
        this.velocity = new Vector2(0,0);
        this.angle = 0;
        this.angularVelocity = 0;
        this.mass = 0;
        this.momentOfInertia = 0;
    }
}
