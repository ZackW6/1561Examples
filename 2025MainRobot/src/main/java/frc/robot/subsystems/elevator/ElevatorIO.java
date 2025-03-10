package frc.robot.subsystems.elevator;

public interface ElevatorIO {
    /**
     * this is a constant call, so once you call it, it will keep going toward that position
     */
    public void setPosition(double position);

    public void stop();

    public double getPosition();

    public double getTarget();
}
