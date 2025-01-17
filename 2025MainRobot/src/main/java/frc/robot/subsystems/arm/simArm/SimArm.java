package frc.robot.subsystems.arm.simArm;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ArmConstants;
import frc.robot.subsystems.arm.ArmIO;
import frc.robot.subsystems.swerve.simSwerve.SimSwerve;

public class SimArm implements ArmIO{

    private final DCMotor gearbox = DCMotor.getFalcon500(1);
    
    private PIDController pidController = new PIDController(110, 0, 7);

    private double targetPosition = 0;

    private boolean stopped = false;

    private Thread updateThread;
    private final SingleJointedArmSim singleJointedArmSim =
      new SingleJointedArmSim(
          gearbox,
          ArmConstants.ARM_ROTOR_TO_SENSOR_RATIO * ArmConstants.ARM_SENSOR_TO_MECHANISM_RATIO,
          SingleJointedArmSim.estimateMOI(ArmConstants.ARM_LENGTH_METERS, ArmConstants.ARM_WEIGHT_KG),
          ArmConstants.ARM_LENGTH_METERS,
          ArmConstants.MIN_ARM_ANGLE_RAD,
          ArmConstants.MAX_ARM_ANGLE_RAD,
          true,
          Units.rotationsToRadians(0)
        );

    public SimArm(){
        updateThread = new Thread(()->{
            while(true){
                try {
                    if (stopped){
                        singleJointedArmSim.setInputVoltage(0);
                    }else{
                        singleJointedArmSim.setInputVoltage(pidController.calculate(getPosition(), targetPosition));
                    }
                    singleJointedArmSim.update(.02);
                    Thread.sleep(20);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
        });
        updateThread.setDaemon(true); 
        updateThread.start();
    }
  
    @Override
    public void setPosition(double position) {
        targetPosition = position;
        stopped = false;
    }

    @Override
    public void stop() {
        stopped = true;
    }

    @Override
    public double getPosition() {
        return (Units.radiansToRotations(singleJointedArmSim.getAngleRads()));
    }

    @Override
    public double getTarget() {
        return targetPosition;
    }
}
