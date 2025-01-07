package frc.robot.hardwareSimUtil.mechanismUtil;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class WheelMech implements MechanismBase{

    private Mechanism2d mech2d;
    private MechanismRoot2d pivot;
    
    private MechanismLigament2d[] spokes;

    public WheelMech(String name, double imageWidth, double imageHeight, double[] pivotPlacement, double spokeLength, int numOfSpokes){
        mech2d = new Mechanism2d(imageWidth, imageHeight);
        pivot = mech2d.getRoot("Pivot", pivotPlacement[0], pivotPlacement[1]);
        spokes = new MechanismLigament2d[numOfSpokes];

        for (int i = 0; i < spokes.length;i++){
            spokes[i] = pivot.append(
            new MechanismLigament2d(
                "Spoke "+i,
                spokeLength,
                (360/spokes.length)*i,
                6,
                new Color8Bit(Color.kAqua)));
        }
        SmartDashboard.putData(name, mech2d);
    }

    @Override
    public void setAngle(double radians) {
        if (spokes[0] != null){
            for (int i = 0; i<spokes.length;i++){
                spokes[i].setAngle(Units.radiansToDegrees(radians)+(360/spokes.length)*i);
            }
        }
    }

}
