package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.constants.GameData;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.ramp.Ramp;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.util.PoseEX;

/**
 * every call pertaining to main scoring subsystems should be called through here
 */
public class CommandMechanism extends BaseMechanism{
    public CommandMechanism(Arm arm, Intake intake, Elevator elevator, Ramp ramp, SwerveDrive swerveDrive){
        super(arm, intake, elevator, ramp, swerveDrive);
    }

    /**
     * 
     * @param coralPose
     * @param heightAbove as in elevator height, takes into account offsets, should be purely how much higher the pivot is from the pose
     * @return
     */
    public double[] scoreStateForPose3d(Pose3d coralPose, double heightAbove) {
        targetPublisher.accept(coralPose);
    
        double armMin = -1.2;
        double armMax = 1.2;
        double armAngleRotations = (armMax + armMin) / 2;
    
        Pose2d pivotPose = swerveDrive.getPose()
            .transformBy(new Transform2d(ELEVATOR_END_EFFECTOR_OFFSETX, 0, new Rotation2d()));
    
        double horizDistance = Math.hypot(coralPose.getX() - pivotPose.getX(), coralPose.getY() - pivotPose.getY());
    
        double targetHeight = heightAbove;
    
        for (int i = 0; i < 30; i++) {
            double tipX = Math.cos(Units.rotationsToRadians(armAngleRotations)) * .26;
            double tipY = Math.sin(Units.rotationsToRadians(armAngleRotations)) * .26;

            double endEffectorRads = Units.rotationsToRadians(armAngleRotations + END_EFFECTOR_SCORE_ANGLE + 0.25);
            double m = Math.tan(endEffectorRads);
    
            double yAtBranch = m * (horizDistance - tipX) + tipY;
    
            double diff = yAtBranch - targetHeight;

            if (horizDistance <= tipX) {
                armAngleRotations = .97;
                break;
            }

            if (diff < 0) {
                armMin = armAngleRotations;
            } else {
                armMax = armAngleRotations;
            }
    
            armAngleRotations = (armMax + armMin) / 2;
        }
        double elevatorHeight = coralPose.getZ() + heightAbove - ELEVATOR_END_EFFECTOR_OFFSETZ;
        return new double[]{armAngleRotations + 0.25 -1, elevatorHeight + .05};
    }
    
    /**
     * applies auto pathing
     * @param target
     * @param straightDist
     * @return
     */
    public Command applyTargetAim(Pose2d intendedRobotPose, Pose3d target, double straightDist, double maxSpeed, double maxRads){
        return reachState(()->scoreStateForPose3d(target, -.03)[0], ()->scoreStateForPose3d(target, -.03)[1]).alongWith(swerveDrive.toPoseAndPoint(intendedRobotPose, target.toPose2d(), straightDist, maxSpeed, maxRads));
    }

    /**
     * does not apply pathplanner pathing
     * @param target
     * @return
     */
    public Command applyTargetAim(Pose2d intendedRobotPose, Pose3d target, double maxSpeed, double maxRads){
        return reachState(()->scoreStateForPose3d(target, -.03)[0], ()->scoreStateForPose3d(target, -.03)[1]).alongWith(swerveDrive.towardPoseWhilePoint(intendedRobotPose, target.toPose2d(), maxSpeed, maxRads));
    }

    public Command backupLowerSafely(){
        return swerveDrive.applyRequest(()->swerveDrive.robotCentricDrive.withSpeeds(new ChassisSpeeds(-.5, 0, 0))).withTimeout(.2)
            .andThen(setState(MainStates.Intake).withTimeout(.2));
    }

    private boolean readyToScore(int place){
        Pose2d pose = GameData.coralPose(place);
        return inState(arm.getTarget(), MAX_ARM_ERROR, elevator.getTarget(), MAX_ELEVATOR_ERROR) 
            && swerveDrive.withinCoords(new Pose2d(pose.getX(), pose.getY()
                , PoseEX.getPoseAngle(swerveDrive.getPose(), GameData.branchPose(place, 1).toPose2d())), .02, .02);
    }

    /**
     * made for auto use, but can be used other if opportunity presents
     * @param autoPath
     * @param place
     * @param offset
     * @param straightDist
     * @return
     */
    public Command autoScoreCoral(Command autoPath, int place, int level, double straightDist){
        return autoPath
            .until(()->swerveDrive.withinCoords(GameData.coralPose(place)
            ,straightDist,1))
            .andThen(Commands.parallel(setCoralState(level), swerveDrive.stop())
                .until(()->inState(arm.getTarget(), .1, elevator.getTarget(), .1)))
            .andThen(Commands.race(applyTargetAim(GameData.coralPose(place)
                ,GameData.branchPose(place, level), 1, 2).until(()->readyToScore(place))
                .andThen(intake.reachGoal(coralShootSpeed).withTimeout(SHOOT_TIME))
            )
        ).andThen(backupLowerSafely());
    }

    public Command autoScoreCoral(int place, int level){
        return autoScoreCoral(swerveDrive.toPose(GameData.coralPose(place)
            , 1.5, 5,3*Math.PI, 5, Math.PI * 3),place, level, 1.5);
    }

    /**
     * made for auto use, but can be used other if opportunity presents
     * @param autoPath
     * @param place
     * @param offset
     * @param straightDist
     * @return
     */
    public Command autoIntakeCoral(Command autoPath, int place, double offset, double straightDist){
        return Commands.race(autoPath.until(()->swerveDrive.withinCoords(GameData.feederPose(place, offset), straightDist, 10))
            .andThen(swerveDrive.towardPose(GameData.feederPose(place, offset), 5,3*Math.PI)), intake());
    }

    public Command autoIntakeCoral(int place, double offset){
        return Commands.race(swerveDrive.toPose(GameData.feederPose(place, offset)
            , 1.2, 5,3*Math.PI), intake());
    }

    public Command autoScoreAlgae(int place){
        return Commands.race(swerveDrive.toPose(GameData.algaeScorePose(place),1.2,5,3*Math.PI),
            Commands.waitUntil(()->swerveDrive.withinCoords(GameData.algaeScorePose(place)
            ,.2,.2,.2))
            .andThen(Commands.race(setAlgaeScoreState(place)
                ,Commands.waitUntil(()->inState(arm.getTarget(), MAX_ARM_ERROR, elevator.getTarget(), MAX_ELEVATOR_ERROR))
                .andThen(intake.reachGoal(algaeShootSpeed).withTimeout(SHOOT_TIME))
                )
            ))
            .andThen(backupLowerSafely());
    }

    public Command autoIntakeAlgae(int place){
        return swerveDrive.toPose(GameData.algaePose(place)
            , 1.2, 5,3*Math.PI)
            .until(()->swerveDrive.withinCoords(GameData.coralPose(place)
            ,.2,.2,.2))
            .andThen(Commands.race(intakeAlgae(place % 2 == 1 ? 1 : 2)
                , swerveDrive.applyRequest(()->swerveDrive.robotCentricDrive.withSpeeds(new ChassisSpeeds(.5, 0, 0))))
            )
            .andThen(backupLowerSafely());
    }
}