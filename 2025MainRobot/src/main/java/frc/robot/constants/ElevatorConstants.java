package frc.robot.constants;

import edu.wpi.first.math.util.Units;

public class ElevatorConstants {
    public static final int ELEVATOR_LEADER_ID = 20;
    public static final int ELEVATOR_FOLLOWER_ID = 21;

    public static final double ELEVATOR_ENCODER_OFFSET = 0;

    public static final double CARRIAGE_POSITION = .2;
    public static final double STAGE_ONE_POSITION = .4;
    public static final double STAGE_TWO_POSITION = .8;

    public static final double P1 = 10;
    public static final double I1 = 0;
    public static final double D1 = 0;

    public static final double S1 = 0.0;
    public static final double G1 = 0;
    public static final double V1 = 0;
    public static final double A1 = 0.0;

    public static final double P2 = 10;
    public static final double I2 = 0;
    public static final double D2 = 0;

    public static final double S2 = 0.0;
    public static final double G2 = 0;
    public static final double V2 = 0;
    public static final double A2 = 0.0;

    public static final double P3 = 10;
    public static final double I3 = 0;
    public static final double D3 = 0;

    public static final double S3 = 0.0;
    public static final double G3 = 0;
    public static final double V3 = 0;
    public static final double A3 = 0.0;
    
    public static final double ELEVATOR_DRUM_RADIUS = .06;
    //64.875 inch rise
    public static final double DRUM_CIRCUMFRANCE_METERS = 2 * Math.PI * ELEVATOR_DRUM_RADIUS;
    public static final double TRUE_ELEVATOR_SENSOR_TO_MECHANISM_RATIO = (25/3.0);
    public static final double ELEVATOR_SENSOR_TO_MECHANISM_RATIO = TRUE_ELEVATOR_SENSOR_TO_MECHANISM_RATIO / DRUM_CIRCUMFRANCE_METERS ;
    public static final double ELEVATOR_ROTOR_TO_SENSOR_RATIO = 1;

    

    public static final double ELEVATOR_MASS_KG = 20;

    public static final double ELEVATOR_MIN_HEIGHT = 0;
    public static final double ELEVATOR_MAX_HEIGHT = 2;

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
