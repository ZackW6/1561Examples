package frc.robot.subsystems.ramp.simRamp;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.constants.ArmConstants;
import frc.robot.constants.RampConstants;
import frc.robot.subsystems.arm.ArmIO;

public class SimRamp implements ArmIO{
    
    //Set what gear and how many gears to use
    private final DCMotor gearbox = DCMotor.getKrakenX60(1);

    //Set PID values
    private PIDController pidController = new PIDController(110, 0, 7);

    private double targetPosition = 0;

    private boolean stopped = false;

    private boolean voltageOut = false;
    private double outputVolts = 0;

    private Thread updateThread;

    //Create new single jointed arm with set vlas
    private final SingleJointedArmSim singleJointedArmSim =
      new SingleJointedArmSim(
          gearbox,
          RampConstants.RAMP_ROTOR_TO_SENSOR_RATIO * RampConstants.RAMP_SENSOR_TO_MECHANISM_RATIO,
          SingleJointedArmSim.estimateMOI(RampConstants.RAMP_LENGTH_METERS, RampConstants.RAMP_WEIGHT_KG),
          RampConstants.RAMP_LENGTH_METERS,
          RampConstants.MIN_RAMP_ANGLE_RAD,
          RampConstants.MAX_RAMP_ANGLE_RAD,
          true,
          Units.rotationsToRadians(0)
        );

    public SimRamp(){
        updateThread = new Thread(()->{
            while(true){
                try {
                    if (stopped){
                        singleJointedArmSim.setInputVoltage(0);
                    }else{
                        if (voltageOut){
                            singleJointedArmSim.setInputVoltage(outputVolts);
                        }else{
                            singleJointedArmSim.setInputVoltage(pidController.calculate(getPosition(), targetPosition));
                        }
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
        voltageOut = false;
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

    @Override
    public void setVoltage(double volts) {
        voltageOut = true;
        stopped = false;
        outputVolts = volts;
    }

}
