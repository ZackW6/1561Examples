package frc.robot.constants;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.motorsims.MapleMotorSim;
import org.ironmaple.simulation.motorsims.SimMotorConfigs;
import org.ironmaple.simulation.motorsims.SimMotorState;
import org.ironmaple.utils.mathutils.MapleCommonMath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;

public class GameData {
    public static final double fieldSizeX = Units.feetToMeters(57.573);
    public static final double fieldSizeY = Units.feetToMeters(26.417);

    public static Pose2d getReefPose(int id, boolean blueOrRed){
        
        return new Pose2d();
    }
}
