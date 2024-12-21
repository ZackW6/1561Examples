package frc.robot.subsystems.motors;

public interface BaseMotor{
  public void set(double x);
  public double getVelocity();
  public double getPosition();
  public double getFixedPosition();
  public int getMotorID();
  public int getEncoderID();
  public BaseMotor setInvert(boolean x);
}