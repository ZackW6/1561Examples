package frc.robot.subsystems.intake.realIntake;

import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.hardware.CANrange;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.subsystems.intake.DigitalInputIO;

public class CANRange implements DigitalInputIO{

    private final CANrange digitalInput;

    private boolean inverted;
    private double minDist;

    public CANRange(int id, double dist){
        minDist = dist;
        digitalInput = new CANrange(id,"Canivore");
        configureCANRange();
    }

    @Override
    public boolean getValue() {
        return (digitalInput.getDistance().getValueAsDouble() <= minDist) ^ inverted;
    }

    /**
     * unused
     */
    @Override
    public void setValue(boolean value) {
        
    }

    @Override
    public void invert(boolean inverted) {
        this.inverted = inverted;
    }

    public void configureCANRange(){
        CANrangeConfiguration configuration = new CANrangeConfiguration();
        digitalInput.getConfigurator().apply(configuration);
    }
    
}
