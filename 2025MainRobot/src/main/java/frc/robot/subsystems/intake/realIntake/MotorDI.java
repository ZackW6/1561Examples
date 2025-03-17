package frc.robot.subsystems.intake.realIntake;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.subsystems.intake.DigitalInputIO;

public class MotorDI implements DigitalInputIO{

    private final BooleanSupplier valueSupplier;

    private boolean inverted;

    public MotorDI(double someValueToCome){
        valueSupplier = ()->someValueToCome > 0;
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
