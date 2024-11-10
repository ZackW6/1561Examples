package frc.robot.simulationUtil.mechanismUtil;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class ArmMech implements MechanismBase{
    private Mechanism2d mech2d;
    private MechanismRoot2d armPivot;
    private MechanismLigament2d armTower;
    private MechanismLigament2d arm;

    public ArmMech(String name, double imageWidth, double imageHeight, double[] pivotPlacement, double armTowerLength, double armTowerAngle, double armLength){
        mech2d = new Mechanism2d(imageWidth, imageHeight);
        armPivot = mech2d.getRoot("ArmPivot", pivotPlacement[0], pivotPlacement[1]);
        armTower = armPivot.append(new MechanismLigament2d("ArmTower", armTowerLength, armTowerAngle));
        arm = armPivot.append(
          new MechanismLigament2d(
              "Arm",
              armLength,
              Units.radiansToDegrees(0),
              6,
              new Color8Bit(Color.kPurple)));
        armTower.setColor(new Color8Bit(Color.kBlue));
        SmartDashboard.putData(name, mech2d);
    }

    @Override
    public void setAngle(double radians){
        arm.setAngle(Units.radiansToDegrees(radians));
    }
}
