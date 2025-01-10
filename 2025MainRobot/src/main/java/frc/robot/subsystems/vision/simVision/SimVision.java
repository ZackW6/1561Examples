/*
 * MIT License
 *
 * Copyright (c) PhotonVision
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

 package frc.robot.subsystems.vision.simVision;
 
 import edu.wpi.first.math.Matrix;
 import edu.wpi.first.math.VecBuilder;
 import edu.wpi.first.math.geometry.Pose2d;
 import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
 import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.LimelightHelpers.RawFiducial;
import frc.robot.Robot;
import frc.robot.constants.LimelightConstants;
import frc.robot.subsystems.swerve.SwerveDriveIO;
import frc.robot.subsystems.swerve.simSwerve.SimSwerve;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.util.PoseEX;

import java.util.List;
 import java.util.Optional;
import java.util.function.Supplier;

import org.photonvision.EstimatedRobotPose;
 import org.photonvision.PhotonCamera;
 import org.photonvision.PhotonPoseEstimator;
 import org.photonvision.PhotonPoseEstimator.PoseStrategy;
 import org.photonvision.simulation.PhotonCameraSim;
 import org.photonvision.simulation.SimCameraProperties;
 import org.photonvision.simulation.VisionSystemSim;
 import org.photonvision.targeting.PhotonTrackedTarget;

import com.ctre.phoenix6.Utils;
 
 public class SimVision extends SubsystemBase implements VisionIO{
     private final PhotonCamera camera;
     private final PhotonPoseEstimator photonEstimator;
     private Matrix<N3, N1> curStdDevs;
 
     // Simulation
     private PhotonCameraSim cameraSim;
     private VisionSystemSim visionSim;

     private final SwerveDriveIO driveIO;
     private boolean completeSim = false;
 
     public SimVision(SwerveDriveIO drivetrain) {
        this.driveIO = drivetrain;
        if (driveIO instanceof SimSwerve){
            completeSim = true;
        }

        camera = new PhotonCamera("main");
        Transform3d last = LimelightConstants.LIMELIGHT_CAMERA_TRANSFORM;
        Transform3d newTransform = new Transform3d(last.getX(), last.getY(), last.getZ(),
             new Rotation3d(last.getRotation().getX(),-last.getRotation().getY(),last.getRotation().getZ()));
        photonEstimator =
                new PhotonPoseEstimator(LimelightConstants.K_TAG_LAYOUT, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, newTransform);
        photonEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

        // ----- Simulation
        if (Robot.isSimulation()) {
            // Create the vision system simulation which handles cameras and targets on the field.
            visionSim = new VisionSystemSim("main");
            // Add all the AprilTags inside the tag layout as visible targets to this simulated field.
            visionSim.addAprilTags(LimelightConstants.K_TAG_LAYOUT);
            // Create simulated camera properties. These can be set to mimic your actual camera.
            var cameraProp = new SimCameraProperties();
            cameraProp.setCalibration(960, 720, Rotation2d.fromDegrees(90));
            cameraProp.setCalibError(0,0);//0.35, 0.10);
            cameraProp.setFPS(15);
            cameraProp.setAvgLatencyMs(0);//50);
            cameraProp.setLatencyStdDevMs(0);//15);
            // Create a PhotonCameraSim which will update the linked PhotonCamera's values with visible
            // targets.
            cameraSim = new PhotonCameraSim(camera, cameraProp);
            // Add the simulated camera to view the targets on this simulated field.
            visionSim.addCamera(cameraSim, newTransform);

            cameraSim.enableDrawWireframe(true);
        }
     }
 
     /**
      * The latest estimated robot pose on the field from vision data. This may be empty. This should
      * only be called once per loop.
      *
      * <p>Also includes updates for the standard deviations, which can (optionally) be retrieved with
      * {@link getEstimationStdDevs}
      *
      * @return An {@link EstimatedRobotPose} with an estimated pose, estimate timestamp, and targets
      *     used for estimation.
      */
     public PoseEstimate getPoseEstimate(String name) {
        
         Optional<EstimatedRobotPose> visionEst = Optional.empty();
         for (var change : camera.getAllUnreadResults()) {
             visionEst = photonEstimator.update(change);
         }
         
         if (visionEst.isEmpty()){
            return new PoseEstimate();
         }
         int size = visionEst.get().targetsUsed.size();
         RawFiducial[] fiducials = new RawFiducial[size];
         int i = 0;
         double totalDist = 0;
         double totalArea = 0;
         double delay = Timer.getFPGATimestamp()-visionEst.get().timestampSeconds;
         for (PhotonTrackedTarget target : visionEst.get().targetsUsed){
            RawFiducial fiducial = new RawFiducial(target.fiducialId,
             target.yaw,
              target.pitch,
               target.area,
                PoseEX.getDistanceFromPoseMeters(driveIO.getPose(), LimelightConstants.K_TAG_LAYOUT.getTagPose(target.fiducialId).get().toPose2d()),
                PoseEX.getDistanceFromPoseMeters(driveIO.getPose(), LimelightConstants.K_TAG_LAYOUT.getTagPose(target.fiducialId).get().toPose2d()),
                 target.poseAmbiguity);
            fiducials[i] = fiducial;
            totalArea+=fiducial.ta;
            totalDist+=fiducial.distToRobot;
            i++;
         }
         PoseEstimate estimate = new PoseEstimate(
            visionEst.get().estimatedPose.toPose2d(),
            Utils.getCurrentTimeSeconds()-delay,
            delay,
            size,
            0,
            totalDist/size,
            totalArea/size,
            fiducials,
            true);

         return estimate;
     }
 
     /**
      * Returns the latest standard deviations of the estimated pose from {@link
      * #getEstimatedGlobalPose()}, for use with {@link
      * edu.wpi.first.math.estimator.SwerveDrivePoseEstimator SwerveDrivePoseEstimator}. This should
      * only be used when there are targets visible.
      */
     public Matrix<N3, N1> getEstimationStdDevs() {
         return curStdDevs;
     }
 
     /** Reset pose history of the robot in the vision system simulation. */
     public void resetSimPose(Pose2d pose) {
         if (Robot.isSimulation()) visionSim.resetRobotPose(pose);
     }


    /**
     * Unused here
     */
    @Override
    public void setOrientation(String name, Rotation2d yaw) {
        
    }

    @Override
    public void periodic() {
        if (completeSim){
            visionSim.update(((SimSwerve)driveIO).getRealPose());
            return;
        }
        visionSim.update(driveIO.getPose());
    }
 }