package frc.robot.subsystems.swerve.simSwerve;

import static edu.wpi.first.units.Units.Grams;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Kilograms;

import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;

public class SimSwerveConstants {
    // Create and configure a drivetrain simulation configuration
    public final static DriveTrainSimulationConfig driveTrainSimulationConfig = DriveTrainSimulationConfig.Default()
                // Specify gyro type (for realistic gyro drifting and error simulation)
                .withGyro(COTS.ofPigeon2())
                // Specify swerve module (for realistic swerve dynamics)
                .withSwerveModule(COTS.ofMark4(
                        DCMotor.getKrakenX60(1), // Drive motor is a Kraken X60
                        DCMotor.getFalcon500(1), // Steer motor is a Falcon 500
                        COTS.WHEELS.COLSONS.cof, // Use the COF for Colson Wheels
                        1)) // L3 Gear ratio
                // Configures the track length and track width (spacing between swerve modules)
                .withTrackLengthTrackWidth(Inches.of(24), Inches.of(24))
                // Configures the bumper size (dimensions of the robot bumper)
                .withBumperSize(Inches.of(30), Inches.of(30))
                .withRobotMass(Kilograms.of(50));
}
