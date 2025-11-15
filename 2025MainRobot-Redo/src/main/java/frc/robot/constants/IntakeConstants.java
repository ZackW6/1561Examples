package frc.robot.constants;

public class IntakeConstants {

    public static final double GEARING = 3;

    public static final int INTAKE_MOTOR_ID = 11;
    public static final int CORAL_LIMIT_SWITCH_ID1 = 1;
    public static final int CORAL_LIMIT_SWITCH_ID2 = 2;
    public static final int CORAL_LASER_ID = 0;
    // Add 0.25 V output to overcome static friction

    public static final double kS = .6; // An error of 1 rps results in 0.11 V output
    public static final double kV = 0; // A velocity target of 1 rps results in 0.12 V output
    public static final double kA = 0; // An acceleration of 1 rps/s requires 0.01 V output
    public static final double kP = .1; // This will need to be tuned after feedforward
    public static final double kI = 0; // For flywheels, this should be 0
    public static final double kD = 0; // For flywheels, this should be 0

    public static final double INTAKE_SENSOR_TO_MECHANISM_RATIO = 2;

    /* Intake Current Limiting */
    public static final int INTAKE_CURRENT_LIMIT = 30;
    public static final int INTAKE_SUPPLY_CURRENT_THRESHOLD = 65;
    public static final int INTAKE_CURRENT_THRESHOLD = 65;
    public static final double INTAKE_CURRENT_THRESHOLD_TIME = 0.1;
    public static final boolean INTAKE_ENABLE_CURRENT_LIMIT = true;

    public static final boolean INTAKE_STATOR_CURRENT_LIMIT_ENABLE = true;
	public static final double INTAKE_STATOR_CURRENT_LIMIT = 60;
}
