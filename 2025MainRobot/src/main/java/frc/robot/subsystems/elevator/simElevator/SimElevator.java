package frc.robot.subsystems.elevator.simElevator;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ElevatorConstants;
import frc.robot.subsystems.elevator.ElevatorIO;

public class SimElevator extends SubsystemBase implements ElevatorIO{

    private final DCMotor gearbox = DCMotor.getFalcon500(2);
    
    private PIDController pidController = new PIDController(5, 0, 0);

    private double targetPosition = 0;

    private boolean stopped = false;

    private final double drumCircumferenceMeters = 2 * Math.PI * ElevatorConstants.ELEVATOR_DRUM_RADIUS;

    private Thread updateThread;

    private final ElevatorSim elevatorSim =
      new ElevatorSim(
          gearbox,
          ElevatorConstants.ELEVATOR_SENSOR_TO_MECHANISM_RATIO * ElevatorConstants.ELEVATOR_ROTOR_TO_SENSOR_RATIO,
          ElevatorConstants.ELEVATOR_MASS_KG,
          ElevatorConstants.ELEVATOR_DRUM_RADIUS,
          ElevatorConstants.ELEVATOR_MIN_HEIGHT,
          ElevatorConstants.ELEVATOR_MAX_HEIGHT,
          true,
          0);
  
    public SimElevator(){
        updateThread = new Thread(()->{
            while(true){
                try {
                    if (stopped){
                        elevatorSim.setInputVoltage(0);
                    }else{
                        elevatorSim.setInputVoltage(pidController.calculate(getPositionMeters(), targetPosition));
                    }
                    elevatorSim.update(.02);
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
        return (elevatorSim.getPositionMeters()/drumCircumferenceMeters);
    }

    @Override
    public double getPositionMeters() {
        return elevatorSim.getPositionMeters();
    }

    @Override
    public double getTargetPositionMeters() {
        return targetPosition;
    }
}
