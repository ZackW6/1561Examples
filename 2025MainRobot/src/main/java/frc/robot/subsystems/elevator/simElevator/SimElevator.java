package frc.robot.subsystems.elevator.simElevator;

import com.ctre.phoenix6.configs.Slot0Configs;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ElevatorConstants;
import frc.robot.subsystems.elevator.ElevatorIO;

public class SimElevator extends SubsystemBase implements ElevatorIO{

    //Set what motor to use and how many motors to sue
    private final DCMotor gearbox = DCMotor.getFalcon500(2);

    //Set what PID vals to use
    private PIDController pidController = new PIDController(50, 0, 0);

    private double targetPosition = 0;

    private boolean stopped = false;

    private boolean voltageOut = false;
    private double outputVolts = 0;

    private Thread updateThread;

    //Create elevator with set values
    private final ElevatorSim elevatorSim =
      new ElevatorSim(
          gearbox,
          ElevatorConstants.TRUE_ELEVATOR_SENSOR_TO_MECHANISM_RATIO * ElevatorConstants.ELEVATOR_ROTOR_TO_SENSOR_RATIO, //The *2 is a lie, sorry
          ElevatorConstants.ELEVATOR_MASS_KG,
          ElevatorConstants.ELEVATOR_DRUM_RADIUS,
          ElevatorConstants.ELEVATOR_MIN_HEIGHT,
          ElevatorConstants.ELEVATOR_MAX_HEIGHT,
          true,
          0);
  
    //Main loop of Elevator
    public SimElevator(){
        updateThread = new Thread(()->{
            while(true){
                try {
                    if (stopped){
                        //Stop Elevator
                        elevatorSim.setInputVoltage(0);
                    }else{
                        //Move elevator based on positions
                        if (voltageOut){
                            elevatorSim.setInputVoltage(outputVolts);
                        }else{
                            elevatorSim.setInputVoltage(pidController.calculate(getPosition(), targetPosition));
                        }
                    }
                    //Update vals every 20 ms
                    elevatorSim.update(.02);
                    //Sleep every 20 ms
                    Thread.sleep(20);
                } catch (InterruptedException e) {
                    //If exception caught, print StackTrace of exceptions
                    e.printStackTrace();
                }
            }
        });
        updateThread.setDaemon(true);
        updateThread.start();
    }
    @Override
    public void setPosition(double position) {
        setPosition(position, 0);
    }

    @Override
    public void setPosition(double position, int slot) {
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
        return elevatorSim.getPositionMeters();
    }

    @Override
    public double getTarget() {
        return targetPosition;
    }

    @Override
    public void setVoltage(double volts) {
        voltageOut = true;
        outputVolts = volts;
        stopped = false;
    }
}
