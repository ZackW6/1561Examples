// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public class ElevatorConstants {
    public static final int ELEVATOR_MOTOR_ID = 12;
    public static final int ELEVATOR_ENCODER_ID = 12;

    public static final double P = 100;
    public static final double I = 0;
    public static final double D = 5;

    public static final double S = 0.0;
    public static final double G = 0.762;
    public static final double V = 0.762;
    public static final double A = 0.0;

    public static final double ELEVATOR_GEARING = 10;
    public static final double ELEVATOR_RADIUS = Units.inchesToMeters(1.0);
    public static final double ELEVATOR_MASS_KG = 1;

    public static final double ELEVATOR_MIN_HEIGHT = 0.25;
    public static final double ELEVATOR_MAX_HEIGHT = 1.25;//these are buggy, deal with it

    public static final double CRUISE_VELOCITY = 400;
    public static final double MAX_ACCELERATION = 1000;
	public static final double JERK = 5000;
}
