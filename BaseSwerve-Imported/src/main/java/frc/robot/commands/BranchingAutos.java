package frc.robot.commands;

import java.io.IOException;
import java.util.Set;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPoint;
import com.pathplanner.lib.util.FileVersionException;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.util.DynamicObstacle;

public class BranchingAutos {

    public static class BranchInstruction{

        static interface GetPiece{
            public String value();
        }

        /**
         * will be based on TBD places by roboducks
         */
        public static enum PiecePose implements GetPiece{
            FirstPiece("I1"),
            SecondPiece("I2"),
            ThirdPiece("I3");

            private final String ID;
            PiecePose(String id) {
                this.ID = id;
            }

            public String value(){
                return ID;
            }
        }

        /**
         * will be based on TBD places by roboducks
         */
        public static enum ShootPose{
            ShootUpper("SU"),
            ShootMiddle("SM"),
            ShootLower("SL");

            private final String ID;
            ShootPose(String id) {
                this.ID = id;
            }

            public String value(){
                return ID;
            }
        }

        /**
         * will be based on TBD places by roboducks
         */
        public static enum BeginPose implements GetPiece{
            BeginUpper("BU"),
            BeginMiddle("BM"),
            BeginLower("BL");

            private final String ID;
            BeginPose(String id) {
                this.ID = id;
            }

            public String value(){
                return ID;
            }
        }


        private final GetPiece from;
        private final ShootPose to;

        /**
         * @param from - piece you have or want to get
         * @param to - where you go to shoot the piece if acquired, else go for next
         */
        public BranchInstruction(GetPiece from, ShootPose to){
            this.from = from;
            this.to = to;
        }

        /**
         * @param from - piece you have or want to get
         * @param to - where you go to shoot the piece if acquired, else go for next
         */
        public static BranchInstruction of(GetPiece from, ShootPose to){
            return new BranchInstruction(from, to);
        }

        public GetPiece getFrom(){
            return from;
        }

        public ShootPose getTo(){
            return to;
        }
    }

    /**
     * this will be mainly of choreo autos, you put in BranchInstruction, and it will figure it out,
     * but make sure the autos exist, and already have bound commands. It is branching as if it cannot 
     * find a piece, it will go to the next one.
     * 
     * Also need auto builder for this, but you probably already did that.
     * 
     * 
     * @apiNote Be cautious, this is complex, it will always be in the format
     * createBranchCommand(whetherOrNotPieceGrabbed,
     *  BranchInstruction.of(BeginPose.BeginUpper, ShootPose.ShootUpper),
     *  BranchInstruction.of(PiecePose.FirstPiece, ShootPose.ShootMiddle),
     *  etc...);
     * 
     * @param reason - this is the boolean supplier saying yes or no to success of intake,
     *  if yes it will go to place/shoot, while if false will continue
     * @param branchInstructions - this is the list of pieces you want to get and where you want to score them.
     */
    public static Command createBranchCommand(BooleanSupplier reason, BranchInstruction... branchInstructions){
        SequentialCommandGroup commandGroup = new SequentialCommandGroup();

        Command firstPath;
        Pose2d startPose;
        
        //Get first path
        firstPath = tryPath(branchInstructions[0].from.value()+branchInstructions[0].to.value());

        try {
            //Find start for odom reset
            PathPlannerPath path = PathPlannerPath.fromChoreoTrajectory(branchInstructions[0].from.value()+branchInstructions[0].to.value());
            startPose = path.getPathPoses().get(0);
        } catch (Exception e) {
            System.out.println(branchInstructions[0].from.value()+branchInstructions[0].to.value()+ " does not exist");
            //TODO, fix this please
            return Commands.none();
        }

        commandGroup.addCommands(firstPath);

        String[] lastToID = new String[]{branchInstructions[0].to.value()};

        for (int i = 1; i < branchInstructions.length; i++){
            String fromIntakeID = branchInstructions[i].from.value();
            String ifShootID = branchInstructions[i].to.value();

            commandGroup.addCommands(Commands.defer(()->tryPath(lastToID[0]+fromIntakeID)
                .andThen(Commands.waitSeconds(.1)), Set.of())
                .andThen(Commands.defer(()->Commands.either(
                    tryPath(fromIntakeID+ifShootID)
                    .alongWith(Commands.runOnce(()->{lastToID[0] = ifShootID;}))
                    ,Commands.runOnce(()->{lastToID[0] = fromIntakeID;})
                    ,reason),Set.of())));
        }

        return AutoBuilder.resetOdom(startPose)
            .andThen(Commands.runOnce(()->{lastToID[0] = branchInstructions[0].to.value();}))
            .andThen(commandGroup);
    }

    private static Command tryPath(String totalID){
        try {
            return AutoBuilder.followPath(PathPlannerPath.fromChoreoTrajectory(totalID));
        } catch (Exception e) {
            System.out.println(totalID +" path does not exist");
            //TODO, eventually this will/should be supplimented by pathplanner autopathing
            return Commands.none();
        }
    }
}