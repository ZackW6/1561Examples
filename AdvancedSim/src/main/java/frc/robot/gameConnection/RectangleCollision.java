package frc.robot.gameConnection;

import com.fasterxml.jackson.annotation.JsonCreator;

import edu.wpi.first.math.geometry.Pose2d;

public class RectangleCollision {

    public static double[] calculateCollision(Rectangle rect1, Rectangle rect2) {
        // Step 1: Since rect2 is static, we assume its velocity is zero
        Vector2 relativeVelocity = rect1.velocity;  // Only rect1 is moving

        // Step 2: Find the vector between the two positions (direction of collision)
        Vector2 collisionNormal = rect1.position.subtract(rect2.position).normalize();

        // Step 3: Calculate the relative velocity along the collision normal
        double relativeVelocityAlongNormal = relativeVelocity.dot(collisionNormal);

        // If the relative velocity along the normal is positive, no collision happens
        if (relativeVelocityAlongNormal > 0) {
            return new double[]{rect1.velocity.x, rect1.velocity.y,0};  // No collision
        }

        // Step 4: Calculate impulse scalar (using elastic collision, coefficient of restitution = 1)
        double e = 1.0; // Elastic collision
        double j = -(1 + e) * relativeVelocityAlongNormal / (1 / rect1.mass);

        // Step 5: Apply impulse to the velocities of rect1
        Vector2 impulse = collisionNormal.multiply(j);
        Vector2 newVelocity1 = rect1.velocity.add(impulse.multiply(1 / rect1.mass));

        // Return the updated velocities of rect1
        return new double[]{newVelocity1.x, newVelocity1.y,0};
    }

}

