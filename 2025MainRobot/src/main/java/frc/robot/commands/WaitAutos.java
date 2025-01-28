package frc.robot.commands;

import java.io.IOException;
import java.util.Set;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.DoubleConsumer;
import java.util.function.Supplier;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPoint;
import com.pathplanner.lib.util.FileVersionException;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.util.DynamicObstacle;

public class WaitAutos {

    public static class BranchInstruction{

        private static interface GetPiece{
            public String value();
        }

        /**
         * will be based on TBD places by roboducks
         */
        public static enum IntakePose implements GetPiece{
            FeederOne("F1"),
            FeederTwo("F2");

            private final String ID;
            IntakePose(String id) {
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
            PlaceA("PA",1),
            PlaceB("PB",2),
            PlaceC("PC",3),
            PlaceD("PD",4),
            PlaceE("PE",5),
            PlaceF("PF",6),
            PlaceG("PG",7),
            PlaceH("PH",8),
            PlaceI("PI",9),
            PlaceJ("PJ",10),
            PlaceK("PK",11),
            PlaceL("PL",12);
            private final String ID;
            private final int num;
            ShootPose(String id, int num) {
                this.ID = id;
                this.num = num;
            }

            public String value(){
                return ID;
            }
        }

        /**
         * will be based on TBD places by roboducks
         */
        public static enum BeginPose implements GetPiece{
            BeginLeft("BL"),
            BeginMiddle("BM"),
            BeginRight("BR");

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
        private final int layer;

        /**
         * @param from - piece you have or want to get
         * @param to - where you go to shoot the piece if acquired, else go for next
         */
        public BranchInstruction(GetPiece from, ShootPose to, int layer){
            this.layer = MathUtil.clamp(layer, 1, 4);
            this.from = from;
            this.to = to;
        }

        /**
         * @param from - piece you have or want to get
         * @param to - where you go to shoot the piece if acquired, else go for next
         */
        public static BranchInstruction of(GetPiece from, ShootPose to, int layer){
            return new BranchInstruction(from, to, layer);
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
     * Make sure to declare factoryCommands beforehand.
     * 
     * @apiNote Be cautious, this is complex, it will always be in the format
     * createBranchCommand(whetherOrNotPieceGrabbed,
     *  BranchInstruction.of(BeginPose.BeginUpper, ShootPose.ShootUpper),
     *  BranchInstruction.of(PiecePose.FirstPiece, ShootPose.ShootMiddle),
     *  etc...);
     * @param beginPose - if you don't have an initial path of that name, it will make one from this pose
     * @param reason - acquired piece or not yet
     * @param branchInstructions - this is the list of pieces you want to get and where you want to score them.
     */
    public static Command createBranchCommand(Pose2d beginPose, BranchInstruction... branchInstructions){
        DynamicObstacle.setDynamicObstacles("avoidAlgae",beginPose.getTranslation());
        SequentialCommandGroup commandGroup = new SequentialCommandGroup();

        Pose2d startPose = beginPose;
        
        //Get first path

        try {
            //Find start for odom reset
            PathPlannerPath path = PathPlannerPath.fromChoreoTrajectory(branchInstructions[0].from.value()+branchInstructions[0].to.value());
            startPose = path.getPathPoses().get(0);
        } catch (Exception e) {
            System.out.println(branchInstructions[0].from.value()+branchInstructions[0].to.value()+ " does not exist");
        }



        for (int i = 0; i < branchInstructions.length; i++){
            String fromID = branchInstructions[i].from.value();
            String shootID = branchInstructions[i].to.value();
            int place = branchInstructions[i].to.num;
            if (i == branchInstructions.length-1){
                commandGroup.addCommands(tryPath(fromID, shootID, place, branchInstructions[i].layer));
                continue;
            }
            commandGroup.addCommands(tryPath(fromID, shootID, place, branchInstructions[i].layer).andThen(tryPath(shootID, branchInstructions[i+1].from.value(), place, 0)));
        }

        return AutoBuilder.resetOdom(startPose)
            .andThen(commandGroup);
    }

    private static Command tryPath(String from, String to, int place, int layer){
        FactoryCommands factoryCommands = FactoryCommands.getInstance().get();
        try {
            return AutoBuilder.followPath(PathPlannerPath.fromChoreoTrajectory(from+to));
        } catch (Exception e) {
            System.out.println(from+to +" path does not exist");
            if (to.startsWith("P")){
                return factoryCommands.autoScoreCoral(place, layer);
            }
            try {
                return factoryCommands.autoIntakeCoral(Integer.parseInt(to.substring(1,2),10));
            } catch (Exception e2) {
                return Commands.none();
            }
            
        }
    }
}