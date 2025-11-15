package frc.robot.subsystems.digitalInputs;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class MotorDI implements DigitalInputIO{

    private final BooleanSupplier valueSupplier;

    private boolean inverted;

    public MotorDI(DoubleSupplier statorCurrent, DoubleSupplier velocityTarget, double min){
        //TODO as of now, this line is specific to knowing about algae, it will need to be altered for other years
        valueSupplier = ()->statorCurrent.getAsDouble() > min && velocityTarget.getAsDouble() < -10;
    }

    @Override
    public boolean getValue() {
        return valueSupplier.getAsBoolean() ^ inverted;
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
