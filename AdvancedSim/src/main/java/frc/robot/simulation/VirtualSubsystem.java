package frc.robot.simulation;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class VirtualSubsystem extends SubsystemBase{
    abstract public void update();

    @Override
    public void periodic() {
        update();
    }
}
