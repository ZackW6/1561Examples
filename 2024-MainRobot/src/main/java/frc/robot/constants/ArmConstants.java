package frc.robot.constants;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Robot;

public class ArmConstants{

        //ONLY IN SIM
        public static final double ARM_WEIGHT_KG = 3;
        public static final double ARM_LENGTH_METERS = .3;
        public static final double ARM_REDUCTION = 32;
        public static final double MAX_ARM_ANGLE_RAD = Units.degreesToRadians(10800);
        public static final double MIN_ARM_ANGLE_RAD = Units.degreesToRadians(-10800);
        public static final double ARM_ENCODER_DIST_PER_PULSE = 2.0 * Math.PI / 4096;

        public static final int ARM_MOTOR_ID = 10;
        public static final int CAN_CODER_ID = 10;

        public static final double kS = 0;
        public static final double kV = 0;
        public static final double kA = 0;
        
        public static final double kP;//300;
        public static final double kI;//.1;
        public static final double kD;//55;
        public static final double kG = 8.574609375;//8.4749;
        static {
            if (Robot.isSimulation()){
                kP = 600;
                kI = 0;
                kD = 0;
            }else{
                kP = 190;
                kI = .2;
                kD = 50;
            }
        }

        // TODO: Make the arm positive when it goes up.
        public static final double CRUISE_VELOCITY = 400;
        public static final double MAX_ACCELERATION = 1000;
		public static final double JERK = 5000;

        
        public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromRotations(.592);//.592);//.063);//.314);//
        

        /* Arm Current Limiting */ //TODO: Change these values
        public static final int ARM_CURRENT_LIMIT = 40;
        public static final int ARM_SUPPLY_CURRENT_THRESHOLD = 65;
        public static final int ARM_CURRENT_THRESHOLD = 70;
        public static final double ARM_CURRENT_THRESHOLD_TIME = 0.1;
        public static final boolean ARM_ENABLE_CURRENT_LIMIT = true;
        public static final boolean ARM_STATOR_CURRENT_LIMIT_ENABLE = true;
		public static final double ARM_STATOR_CURRENT_LIMIT = 200;
        
    }
