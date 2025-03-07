package frc.robot.constants;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.util.Units;
// import frc.robot.util.MultiLinearInterpolator;
import frc.robot.util.MultiLinearInterpolator;

public class LimelightConstants {
    /* 5027 https://github.com/FRC5727/SwervyBoi/blob/76bf195e5332ee201a1d0d766fbc0b57b428d485/src/main/java/frc/robot/Constants.java */
    public static final String BACKWARD_LIMELIGHT_NAME = "limelight-backcam";
    public static final double MAX_XY_ERROR = 1.0;
    public static final Transform3d BACKWARD_LIMELIGHT_CAMERA_TRANSFORM =
        new Transform3d(new Translation3d(0,-0.297, 0.25146), new Rotation3d(0, Units.degreesToRadians(0), Units.degreesToRadians(180)));
    public static final String FR_LIMELIGHT_NAME = "limelight-rfront";
    public static final Transform3d FR_LIMELIGHT_CAMERA_TRANSFORM =
        new Transform3d(new Translation3d(0.297,0.297, 0.25146), new Rotation3d(180, Units.degreesToRadians(10), Units.degreesToRadians(30)));

    public static final String FL_LIMELIGHT_NAME = "limelight-lfront";
    public static final Transform3d FL_LIMELIGHT_CAMERA_TRANSFORM =
        new Transform3d(new Translation3d(-0.297,0.297, 0.25146), new Rotation3d(180, Units.degreesToRadians(10), Units.degreesToRadians(-30)));
    // public static final String AMP_CAM = "limelight-object";
    // public static final double MAX_XY_ERROR_AMP_CAM = 1.0;
    // public static final Transform3d AMP_CAM_TRANSFORM =
    //     new Transform3d(new Translation3d(0,-.101, .522), new Rotation3d(0, Units.degreesToRadians(-20), Units.degreesToRadians(0)));

    public static final AprilTagFieldLayout K_TAG_LAYOUT = AprilTagFields.k2025Reefscape.loadAprilTagLayoutField();

    public static final double[][] ONE_APRIL_TAG_LOOKUP_TABLE = {
      // {distance in meters, x std deviation, y std deviation, r (in degrees//actually maybe radians) std deviation}
      {0, 0.0, 0.0, .2},
      {1.5, 0.0, 0.0, .3},
      {3, .25, .25, .4},
      {4.5, 1, 1, .6},
      {6, 2, 2, 2},
      {7, 999, 999, 999}
    };
    
    public static final MultiLinearInterpolator ONE_APRIL_TAG_LINEAR_INTERPOLATOR = new MultiLinearInterpolator(ONE_APRIL_TAG_LOOKUP_TABLE);

    public static final double[][] ROTATION_LOOKUP_TABLE = {
      // {distance in meters, x std deviation, y std deviation, r (in degrees) std deviation}
      {0,.1},
      {1.5, .25},
      {3,.5},
      {4.5, 3},
      {6, 1000}
    };
    
    public static final MultiLinearInterpolator ROTATION_LINEAR_INTERPOLATOR = new MultiLinearInterpolator(ROTATION_LOOKUP_TABLE);

    public static final double[][] AUTO_LOOKUP_TABLE = {
      // {distance in meters, x std deviation, y std deviation, r (in degrees//actually maybe radians) std deviation}
      {0, .2, .2, 999999},
      {1.5, .3, .3, 999999},
      {3, .4, .4, 999999},
      {4.5, 1, 1, 999999},
      {6, 2, 2, 999999}
    };

    public static final MultiLinearInterpolator AUTO_INTERPOLATOR = new MultiLinearInterpolator(AUTO_LOOKUP_TABLE);

}
