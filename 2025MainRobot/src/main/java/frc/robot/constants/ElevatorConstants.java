package frc.robot.constants;

import edu.wpi.first.math.util.Units;

public class ElevatorConstants {
    public static final int ELEVATOR_LEADER_ID = 12;
    public static final int ELEVATOR_FOLLOWER_ID = 13;
    public static final int ELEVATOR_ENCODER_ID = 12;

    public static final double ELEVATOR_ENCODER_OFFSET = 0;

    public static final double P = 100;
    public static final double I = 0;
    public static final double D = 5;

    public static final double S = 0.0;
    public static final double G = 0.762;
    public static final double V = 0.762;
    public static final double A = 0.0;

    public static final double ELEVATOR_SENSOR_TO_MECHANISM_RATIO = 1;
    public static final double ELEVATOR_ROTOR_TO_SENSOR_RATIO = 10;

    public static final double ELEVATOR_DRUM_RADIUS = Units.inchesToMeters(2.0);

    public static final double ELEVATOR_MASS_KG = 20;

    public static final double ELEVATOR_MIN_HEIGHT = 0;
    public static final double ELEVATOR_MAX_HEIGHT = 3;

    public static final double CRUISE_VELOCITY = 400;
    public static final double MAX_ACCELERATION = 1000;
	public static final double JERK = 5000;

    /* Elevator Current Limiting */ //TODO: Change/Fix these values
    public static final int ELEVATOR_CURRENT_LIMIT = 40;
    public static final int ELEVATOR_SUPPLY_CURRENT_THRESHOLD = 65;
    public static final int ELEVATOR_CURRENT_THRESHOLD = 70;
    public static final double ELEVATOR_CURRENT_THRESHOLD_TIME = 0.1;
    public static final boolean ELEVATOR_ENABLE_CURRENT_LIMIT = true;
    public static final boolean ELEVATOR_STATOR_CURRENT_LIMIT_ENABLE = true;
    public static final double ELEVATOR_STATOR_CURRENT_LIMIT = 120;
}
