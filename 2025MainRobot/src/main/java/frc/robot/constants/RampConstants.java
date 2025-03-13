package frc.robot.constants;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Robot;

public class RampConstants{

    public static final double RAMP_WEIGHT_KG = 3;
    public static final double RAMP_LENGTH_METERS = .36;
    public static final double MAX_RAMP_ANGLE_RAD = Units.rotationsToRadians(.5);
    public static final double MIN_RAMP_ANGLE_RAD = Units.rotationsToRadians(-.5);
    public static final double RAMP_ENCODER_DIST_PER_PULSE = 2.0 * Math.PI / 4096;

    public static final int RAMP_MOTOR_ID = 30;

    public static final double kS = 0;
    public static final double kV = 0;
    public static final double kA = 0;
    
    public static final double kP = 10;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double kG = 0;//8.4749;

    public static final double RAMP_SENSOR_TO_MECHANISM_RATIO = 10;
    public static final double RAMP_ROTOR_TO_SENSOR_RATIO = 1;


    // TODO: Make the RAMP positive when it goes up.
    public static final double CRUISE_VELOCITY = 400;
    public static final double MAX_ACCELERATION = 1000;
    public static final double JERK = 5000;

    
    public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromRotations(0);
    
    public static final double JOINT_START_HEIGHT_METERS = .2;
    public static final double RAMP_END_DEFFECTOR_SCORE_ANGLE = 0.05;//Rotations
    public static final double RAMP_END_DEFFECTOR_SCORE_OFFSET = -Units.degreesToRotations(32.8);//Rotations

    /* RAMP Current Limiting */ //TODO: Change/Fix these values
    public static final int RAMP_CURRENT_LIMIT = 40;
    public static final int RAMP_SUPPLY_CURRENT_THRESHOLD = 65;
    public static final int RAMP_CURRENT_THRESHOLD = 70;
    public static final double RAMP_CURRENT_THRESHOLD_TIME = 0.1;
    public static final boolean RAMP_ENABLE_CURRENT_LIMIT = true;
    public static final boolean RAMP_STATOR_CURRENT_LIMIT_ENABLE = true;
    public static final double RAMP_STATOR_CURRENT_LIMIT = 120;
    
}
