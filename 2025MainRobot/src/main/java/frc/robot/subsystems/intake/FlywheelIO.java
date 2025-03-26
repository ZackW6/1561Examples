package frc.robot.subsystems.intake;

public interface FlywheelIO {
    /**
     * this is a constant call, so once you call it, it will keep going toward that position. In rotations per second
     * 
     */
    public void setVelocity(double rps);

    public void setVoltage(double volts);

    public void stop();

    /**
     * in mechinism rotations per second
     * @return
     */
    public double getVelocity();

    public double getTarget();

    public double getCurrent();

    public double getAcceleration();
}
