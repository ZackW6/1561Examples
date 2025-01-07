package frc.robot.subsystems.swerve.simSwerve;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.SwerveDriveIO;

public class SimSwerve implements SwerveDriveIO{
    private final SwerveDriveSimulation swerveDriveSimulation;
    public SimSwerve(){
        swerveDriveSimulation = SimSwerveConstants.getSimSwerve();
        SimulatedArena.getInstance().addDriveTrainSimulation(swerveDriveSimulation);
    }
    @Override
    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'applyRequest'");
    }

    @Override
    public Pose2d getPose() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getPose'");
    }

    @Override
    public void configureTeleop(DoubleSupplier vx, DoubleSupplier vy, DoubleSupplier vrot) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'configureTeleop'");
    }

    @Override
    public Command addFieldRelativeSpeeds(DoubleSupplier vx, DoubleSupplier vy, DoubleSupplier vr, String key) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'addFieldRelativeSpeeds'");
    }

    @Override
    public Command addFieldFacingSpeeds(DoubleSupplier vx, DoubleSupplier vy, DoubleSupplier vr, String key) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'addFieldFacingSpeeds'");
    }

    @Override
    public Command addRobotRelativeSpeeds(DoubleSupplier vx, DoubleSupplier vy, DoubleSupplier vr, String key) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'addRobotRelativeSpeeds'");
    }

    @Override
    public void addFieldRelativeSpeeds(double vx, double vy, double vr, String key) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'addFieldRelativeSpeeds'");
    }

    @Override
    public void addFieldFacingSpeeds(double vx, double vy, double vr, String key) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'addFieldFacingSpeeds'");
    }

    @Override
    public void addRobotRelativeSpeeds(double vx, double vy, double vr, String key) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'addRobotRelativeSpeeds'");
    }

    @Override
    public void removeSource(String key) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'removeSource'");
    }

    @Override
    public void configurePathPlanner() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'configurePathPlanner'");
    }

    @Override
    public Command getAutoPath(String pathName) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getAutoPath'");
    }

    @Override
    public ChassisSpeeds getCurrentRobotChassisSpeeds() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getCurrentRobotChassisSpeeds'");
    }

    @Override
    public Rotation2d getYaw() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getYaw'");
    }

    @Override
    public Rotation2d getYawOffset() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getYawOffset'");
    }
    
}
