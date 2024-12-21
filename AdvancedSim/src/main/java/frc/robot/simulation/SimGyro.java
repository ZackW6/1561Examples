package frc.robot.simulation;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Robot;

public class SimGyro {
    private Pigeon2 gyro = null;
    private Rotation2d simRot = new Rotation2d();
    public SimGyro(int id){
        if (Robot.isSimulation()){
            return;
        }
        gyro = new Pigeon2(id,"Canivore");
    }

    public Rotation2d getRotation(){
        if (Robot.isSimulation()){
            return simRot;
        }
        return gyro.getRotation2d();
    }

    public void setRotation(Rotation2d rot){
        simRot = rot;
    }
}
