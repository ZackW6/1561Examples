// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public class ElevatorConstants {
    public static final int ELEVATOR_MOTOR1_ID = 12;
    public static final int ELEVATOR_MOTOR2_ID = 11;

    public static final int ELEVATOR_ENCODER_ID = 12;

    public static final double P = 160;
    public static final double I = 0;
    public static final double D = 0;

    public static final double S = 0.0;
    public static final double G = .762;
    public static final double V = .762;
    public static final double A = 0.0;

    public static final double SPROCKET_RADIUS = Units.inchesToMeters(0.814);
    public static final double ELEVATOR_GEARING = 7.75/(2*Math.PI*SPROCKET_RADIUS);//10
    public static final double ELEVATOR_RADIUS = Units.inchesToMeters(1);

    public static final double MAX_HEIGHT = Units.feetToMeters(1);

    // Values used in sim only
    public static final double ELEVATOR_MASS_KG = 4; // kg

    // Encoder is reset to measure 0 at the bottom, so minimum height is 0.
    public static final double ELEVATOR_MIN_HEIGHT = 0.0;
    public static final double ELEVATOR_MAX_HEIGHT = 2;//1.25;

    public static final double CRUISE_VELOCITY = 400;
    public static final double MAX_ACCELERATION = 1000;
	public static final double JERK = 5000;
}
