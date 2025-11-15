package frc.robot.subsystems.digitalInputs;

public interface DigitalInputIO {
    public boolean getValue();

    /**
     * only for sim
     */
    public void setValue(boolean value);

    public void invert(boolean inverted);
}
