package frc.robot.subsystems.intake.realIntake;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.subsystems.intake.DigitalInputIO;

public class DigitalInputLS implements DigitalInputIO{

    private final DigitalInput digitalInput;

    private boolean inverted;

    public DigitalInputLS(int id){
        digitalInput = new DigitalInput(id);
    }

    @Override
    public boolean getValue() {
        return digitalInput.get() ^ inverted;
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
    
}
