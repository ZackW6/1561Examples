package frc.robot.gameConnection;

import com.fasterxml.jackson.annotation.JsonCreator;

public class Vector2 {
    public double x;
    public double y;

    @JsonCreator
    Vector2(double x, double y) {
        this.x = x;
        this.y = y;
    }

    Vector2 add(Vector2 other) {
        return new Vector2(this.x + other.x, this.y + other.y);
    }

    Vector2 subtract(Vector2 other) {
        return new Vector2(this.x - other.x, this.y - other.y);
    }

    Vector2 multiply(double scalar) {
        return new Vector2(this.x * scalar, this.y * scalar);
    }

    double dot(Vector2 other) {
        return this.x * other.x + this.y * other.y;
    }

    double cross(Vector2 other) {
        return this.x * other.y - this.y * other.x;
    }

    Vector2 normalize() {
        double magnitude = Math.sqrt(x * x + y * y);
        return new Vector2(x / magnitude, y / magnitude);
    }
}
