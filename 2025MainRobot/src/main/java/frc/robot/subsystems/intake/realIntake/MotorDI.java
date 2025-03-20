package frc.robot.subsystems.intake.realIntake;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.subsystems.intake.DigitalInputIO;

public class MotorDI implements DigitalInputIO{

    private final BooleanSupplier valueSupplier;

    private boolean inverted;

    public MotorDI(DoubleSupplier statorCurrent, DoubleSupplier motorAcceleration, double min){
        valueSupplier = ()->statorCurrent.getAsDouble() > min && motorAcceleration.getAsDouble() > -10;
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
