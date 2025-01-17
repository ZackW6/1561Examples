package frc.robot.subsystems.intake.simIntake;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.constants.ArmConstants;
import frc.robot.constants.IntakeConstants;
import frc.robot.subsystems.intake.FlywheelIO;

public class SimIntake implements FlywheelIO{
    private final DCMotor gearbox = DCMotor.getFalcon500(1);
    
    private PIDController pidController = new PIDController(5, 0, 0);

    private double targetVelocity = 0;

    private boolean stopped = false;

    private Thread updateThread;
    private final FlywheelSim intakeSim = new FlywheelSim(LinearSystemId.createFlywheelSystem(gearbox,0.00032,IntakeConstants.GEARING),
     gearbox,
      0);

    public SimIntake(){
        updateThread = new Thread(()->{
            while(true){
                try {
                    if (stopped){
                        intakeSim.setInputVoltage(0);
                    }else{
                        intakeSim.setInputVoltage(pidController.calculate(getVelocity(), targetVelocity));
                    }
                    intakeSim.update(.02);
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
    public void setVelocity(double rps) {
        targetVelocity = rps;
        stopped = false;
    }

    @Override
    public void stop() {
        stopped = true;
    }

    /**
     * in rotations of mechanism per second
     */
    @Override
    public double getVelocity() {
        return (Units.radiansToRotations(intakeSim.getAngularVelocityRadPerSec()));
    }

    @Override
    public double getTarget() {
        return targetVelocity;
    }
}
