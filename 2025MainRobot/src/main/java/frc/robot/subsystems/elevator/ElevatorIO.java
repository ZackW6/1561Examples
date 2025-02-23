package frc.robot.subsystems.elevator;

public interface ElevatorIO {
    /**
     * this is a constant call, so once you call it, it will keep going toward that position
     */
    public void setPosition(double position);

    public void stop();

    /**
     * of mechanism rotations, not encoder or motor
     * @return
     */
    public double getPosition();

    public double getPositionMeters();

    public double getTargetPositionMeters();

    public void assignPID(double P, double I, double D);

    //Assign Static, Gravitational, Velocity, Acceleration FeedForward
    public void assignSGVA(double S, double G, double V, double A);

    public double[] recievePIDs();
}
