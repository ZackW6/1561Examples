package frc.robot.subsystems.vision.objectDetection;

import edu.wpi.first.math.geometry.Rotation2d;

public interface ObjectDetectionIO {

    public Rotation2d getHorizontalRotationFromTarget();

    public Rotation2d getVerticalRotationFromPiece();

    public boolean isPiecePresent();
}
