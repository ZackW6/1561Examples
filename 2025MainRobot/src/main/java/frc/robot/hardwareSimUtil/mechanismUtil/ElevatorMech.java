package frc.robot.hardwareSimUtil.mechanismUtil;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class ElevatorMech implements MechanismBase{

    private Mechanism2d mech2d;
    private MechanismRoot2d elevatorRoot;
    private MechanismLigament2d elevatorLigament;

    private double[] pose;

    /**
     * all doubles are in meters
     * @param name
     * @param ligamentLength
     * @param ligamentWidth
     */
    public ElevatorMech(String name, double imageWidth, double imageHeight, double[] elevatorInitPos, double ligamentLength){
        mech2d = new Mechanism2d(imageWidth, imageHeight);
        elevatorRoot = mech2d.getRoot("Elevator Root", elevatorInitPos[0], elevatorInitPos[1]);
        elevatorLigament =
            elevatorRoot.append(
                new MechanismLigament2d("Elevator", ligamentLength, 90));
        elevatorLigament.setColor(new Color8Bit(Color.kCrimson));
        elevatorLigament.setLineWeight(30);
                SmartDashboard.putData(name, mech2d);
        this.pose = elevatorInitPos;
    }

    /**
     * in this case it is position meters, ignore some of the stuff that normally has to do with this
     */
    @Override
    public void setAngle(double meters) {
        pose[1] = meters;
        elevatorRoot.setPosition(pose[0], pose[1]-.25);
    }
    
}
