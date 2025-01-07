package frc.robot.subsystems.swerve;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.swerve.swerveHelpers.MainDrive;

public class SwerveDrive extends SubsystemBase{
    private final MainDrive mainRequest;

    private final Vision cameras;

    private final SwerveDriveIO swerveIO;

    public SwerveDrive(SwerveDriveIO swerveIO){
        this.swerveIO = swerveIO;
        
    }
}
