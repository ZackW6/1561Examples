package frc.robot.subsystems.climb.simClimber;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.constants.ArmConstants;
import frc.robot.constants.ClimberConstants;
import frc.robot.subsystems.arm.ArmIO;

public class SimClimber implements ArmIO{

    //Set what motor to use and how many motors to use
    private final DCMotor gearbox = DCMotor.getKrakenX60(1);

    //Set PID vals
    private PIDController pidController = new PIDController(110, 0, 7);

    private double targetPosition = 0;

    private boolean stopped = false;

    private Thread updateThread;

    //Created SingleJointedArmSim with set values
    private final SingleJointedArmSim singleJointedArmSim =
      new SingleJointedArmSim(
          gearbox,
          ClimberConstants.CLIMBER_ROTOR_TO_SENSOR_RATIO * ClimberConstants.CLIMBER_SENSOR_TO_MECHANISM_RATIO,
          SingleJointedArmSim.estimateMOI(ClimberConstants.CLIMBER_LENGTH_METERS, ClimberConstants.CLIMBER_WEIGHT_KG),
          ClimberConstants.CLIMBER_LENGTH_METERS,
          ClimberConstants.MIN_CLIMBER_ANGLE_RAD,
          ClimberConstants.MAX_CLIMBER_ANGLE_RAD,
          true,
          Units.rotationsToRadians(0)
        );

    //Main Loop of SimClimber
    public SimClimber(){
        updateThread = new Thread(()->{
            while(true){
                try {
                    if (stopped){
                        //Stop motor
                        singleJointedArmSim.setInputVoltage(0);
                    }else{
                        //Move arm based on Positions
                        singleJointedArmSim.setInputVoltage(pidController.calculate(getPosition(), targetPosition));
                    }
                    //Update Vals every 20 ms
                    singleJointedArmSim.update(.02);
                    //Sleep every ms
                    Thread.sleep(20);
                //If exception is caught, print stackTrace of Exceptions
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
