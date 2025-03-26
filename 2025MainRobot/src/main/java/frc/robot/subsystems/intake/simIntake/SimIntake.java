package frc.robot.subsystems.intake.simIntake;

import static edu.wpi.first.units.Units.Inch;
import static edu.wpi.first.units.Units.Meter;

import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.SimulatedArena;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.constants.ArmConstants;
import frc.robot.constants.IntakeConstants;
import frc.robot.subsystems.intake.FlywheelIO;
import frc.robot.subsystems.swerve.simSwerve.SimSwerve;

public class SimIntake implements FlywheelIO{

    //Set what type of gear and how many gears to use
    private final DCMotor gearbox = DCMotor.getFalcon500(1);
    
    //Set PID values
    private PIDController pidController = new PIDController(20, 0, 0);

    private double targetVelocity = 0;

    private boolean stopped = false;

    private boolean voltageOut = false;
    private double outputVolts = 0;

    private Thread updateThread;

    //Create new intake with set values
    private final FlywheelSim intakeSim = new FlywheelSim(LinearSystemId.createFlywheelSystem(gearbox,0.012,IntakeConstants.GEARING),
     gearbox,
      0);

    public SimIntake(){
        updateThread = new Thread(()->{
            while(true){
                try {
                    if (stopped){
                        //Stop intake
                        intakeSim.setInputVoltage(0);
                    }else{
                        //Run intake with set values
                        if (voltageOut){
                            intakeSim.setInputVoltage(outputVolts);
                        }else{
                            intakeSim.setInputVoltage(pidController.calculate(getVelocity(), targetVelocity));
                        }
                    }
                    //Update every 20 ms
                    intakeSim.update(.02);
                    //Sleep every 20 ms
                    Thread.sleep(20);
                } catch (InterruptedException e) {
                    //Print StackTrace if exception is caught
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
        voltageOut = false;
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

    @Override
    public double getCurrent() {
        return intakeSim.getCurrentDrawAmps();
    }

    @Override
    public double getAcceleration() {
        return (Units.radiansToRotations(intakeSim.getAngularAccelerationRadPerSecSq()));
    }

    @Override
    public void setVoltage(double volts) {
        voltageOut = true;
        outputVolts = volts;
        stopped = false;
    }
}
