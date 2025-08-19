package frc.robot;

import java.util.List;
import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import com.revrobotics.ColorSensorV3.GainFactor;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.GameData;
import frc.robot.subsystems.CommandMechanism;
import frc.robot.subsystems.BaseMechanism.MainStates;
import frc.robot.util.CustomController;
import frc.robot.util.PoseEX;
import frc.robot.util.SendableConsumer;

public class OptionController {

    private int reefLevel = 4;
    private int position = 1;

    private int closestSide = 1;
    private int closestFeeder = 1;

    private int defaultFeeder = 1;

    private int feederPosition = 2;

    private int algaeScoreLevel = 1;

    private final CommandMechanism mainMechanism;

    private final BooleanSupplier hasCoral;

    private final BooleanSupplier hasAlgae;

    private final Supplier<Pose2d> robotPose;

    private final Notifier notifier;

    public OptionController(CustomController controller, CommandMechanism mainMechanism, BooleanSupplier hasCoral, BooleanSupplier hasAlgae, Supplier<Pose2d> robotPose){

        this.mainMechanism = mainMechanism;
        this.hasCoral = hasCoral;
        this.hasAlgae = hasAlgae;

        notifier = new Notifier(this :: periodic);
        notifier.setName("Scoring Periodic");
        notifier.startPeriodic(.02);
        Runtime.getRuntime().addShutdownHook(new Thread(notifier::close));

        this.robotPose = robotPose;

        // controller.fixedButtonPressed(1).onTrue(Commands.runOnce(()->{
        //     //Intended to be A scoring
        //     position = 1;
        // }));
        // controller.fixedButtonPressed(2).onTrue(Commands.runOnce(()->{
        //     //Intended to be B scoring
        //     position = 2;
        // }));
        // controller.fixedButtonPressed(3).onTrue(Commands.runOnce(()->{
        //     //Intended to be C scoring
        //     position = 3;
        // }));
        // controller.fixedButtonPressed(4).onTrue(Commands.runOnce(()->{
        //     //Intended to be D scoring
        //     position = 4;
        // }));
        // controller.fixedButtonPressed(5).onTrue(Commands.runOnce(()->{
        //     //Intended to be E scoring
        //     position = 5;
        // }));
        // controller.fixedButtonPressed(6).onTrue(Commands.runOnce(()->{
        //     //Intended to be F scoring
        //     position = 6;
        // }));
        // controller.fixedButtonPressed(7).onTrue(Commands.runOnce(()->{
        //     //Intended to be G scoring
        //     position = 7;
        // }));
        // controller.fixedButtonPressed(8).onTrue(Commands.runOnce(()->{
        //     //Intended to be H scoring
        //     position = 8;
        // }));
        // controller.fixedButtonPressed(9).onTrue(Commands.runOnce(()->{
        //     //Intended to be I scoring
        //     position = 9;
        // }));
        // controller.fixedButtonPressed(10).onTrue(Commands.runOnce(()->{
        //     //Intended to be J scoring
        //     position = 10;
        // }));
        // controller.fixedButtonPressed(11).onTrue(Commands.runOnce(()->{
        //     //Intended to be K scoring
        //     position = 11;
        // }));
        // controller.fixedButtonPressed(12).onTrue(Commands.runOnce(()->{
        //     //Intended to be L scoring
        //     position = 12;
        // }));
        controller.fixedButtonPressed(13).onTrue(Commands.runOnce(()->{
            //Intended to be L1 scoring
            reefLevel = 1;
        }));
        controller.fixedButtonPressed(14).onTrue(Commands.runOnce(()->{
            //Intended to be L2 scoring
            reefLevel = 2;
        }));
        controller.fixedButtonPressed(15).onTrue(Commands.runOnce(()->{
            //Intended to be L3 scoring
            reefLevel = 3;
        }));
        controller.fixedButtonPressed(16).onTrue(Commands.runOnce(()->{
            //Intended to be L4 scoring
            reefLevel = 4;
        }));
        controller.fixedButtonPressed(17).onTrue(Commands.runOnce(()->{
            //Intended to be processor scoring
            algaeScoreLevel = 1;
        }));
        controller.fixedButtonPressed(18).onTrue(Commands.runOnce(()->{
            //Intended to be processor scoring
            algaeScoreLevel = 2;
        }));
        // controller.fixedButtonPressed(19).onTrue(Commands.runOnce(()->{
        //     //Intended to be feeder 1 intake
        //     defaultFeeder = 1;
        // }));
        // controller.fixedButtonPressed(20).onTrue(Commands.runOnce(()->{
        //     //Intended to be feeder 2 intake
        //     defaultFeeder = 2;
        // }));

        controller.fixedButtonPressed(21).onTrue(Commands.runOnce(()->{
            //Intended to be feeder to the left offset
            feederPosition = 1;
        })).and(()->controller.getFixedButton(22)).onTrue(Commands.runOnce(()->{
            feederPosition = 2;
        }));
        controller.fixedButtonPressed(22).onTrue(Commands.runOnce(()->{
            //Intended to be feeder to the right offset
            feederPosition = 3;
        })).and(()->controller.getFixedButton(21)).onTrue(Commands.runOnce(()->{
            feederPosition = 2;
        }));

        // SendableConsumer.createSendableChooser("Place", this::setPlace, 1);
    }

    // public void setPlace(double place){
    //     reefPosition = (int)place;
    // }

    /**
     * auto intakes a coral if it doesn't have one, if it has an algae it scores it
     * @return
     */
    public Command getAutoIntake(){
        return Commands.defer(()->Commands.either(algaeTillInterruptSwap()
        ,Commands.either(coralTillInterruptSwap()
            , coralIntakeTillInterruptSwap()
            , hasCoral)
        ,hasAlgae), mainMechanism.mainSubsystems);
    }

    // /**
    //  * scores coral automatically if it has coral, if it does not it goes and grabs an algae, if it had an algae, it scores.
    //  * @return
    //  */
    // public Command getAutoScore(){
    //     return Commands.defer(()->Commands.either(coralTillInterruptSwap()
    //         , Commands.either(algaeTillInterruptSwap()
    //             , algaeIntakeTillInterruptSwap()
    //             , hasAlgae)
    //         ,hasCoral),factoryCommands.mainSubsytems);
    // }

    /**
     * 
     * @return
     */
    public Command getAutoCoral(){
        return getAutoCoral(position);
    }

    /**
     * position as in left or right position of the reef
     * @return
     */
    public Command getAutoCoral(int position){
        return Commands.defer(()->Commands.either(algaeTillInterruptSwap()
        ,Commands.either(coralTillInterruptSwap(position)
            , coralIntakeTillInterruptSwap()
            , hasCoral)
        ,hasAlgae),mainMechanism.mainSubsystems).andThen(Commands.defer(()->getAutoCoral(position), mainMechanism.mainSubsystems));
    }

    /**
     * 
     * @return
     */
    public Command getAutoAlgae(){
        return Commands.defer(()->Commands.either(coralTillInterruptSwap()
            , Commands.either(algaeTillInterruptSwap()
                , algaeIntakeTillInterruptSwap()
                , hasAlgae)
            ,hasCoral),mainMechanism.mainSubsystems).andThen(Commands.defer(()->getAutoAlgae(), mainMechanism.mainSubsystems));
    }

    private Command coralTillInterruptSwap(){
        return coralTillInterruptSwap(position);
    }

    private Command coralTillInterruptSwap(int sidePosition){
        return Commands.defer(()->{
            int initPosition = sidePosition;
            int initReefPosition = closestSide * 2 -2 + initPosition;
            int initReefLevel = reefLevel;
            return mainMechanism.autoScoreCoral(initReefPosition, initReefLevel)
                .until(()->initReefPosition != closestSide * 2 -2 + initPosition || (initReefLevel != reefLevel))
                .andThen(coralTillInterruptSwap(initPosition)).unless(()->!hasCoral.getAsBoolean());
        },Set.of());
    }

    private Command coralIntakeTillInterruptSwap(){
        return Commands.defer(()->{
            int initFeederDefault = closestFeeder;
            int initFeederPosition = feederPosition;
            return mainMechanism.autoIntakeCoral(initFeederDefault, (initFeederPosition-2)*GameData.optionalFeederRightOffset)
                .until(()->initFeederDefault != closestFeeder || initFeederPosition != feederPosition)
                .andThen(coralIntakeTillInterruptSwap()).unless(hasCoral);
        },Set.of());
    }

    private Command algaeTillInterruptSwap(){
        return Commands.defer(()->{
            int initAlgaeLevel = algaeScoreLevel;
            return mainMechanism.autoScoreAlgae(initAlgaeLevel)
                .until(()->initAlgaeLevel != algaeScoreLevel)
                .andThen(algaeTillInterruptSwap()).unless(()->!hasAlgae.getAsBoolean());
        },Set.of());
    }

    private Command algaeIntakeTillInterruptSwap(){
        return Commands.defer(()->{
            int initReefPosition = closestSide;
            return mainMechanism.autoIntakeAlgae(closestSide)
                .until(()->initReefPosition != closestSide)
                .andThen(algaeIntakeTillInterruptSwap()).unless(hasAlgae);
        },Set.of());
    }

    public Command getScoreLevel(){
        return Commands.defer(()->mainMechanism.setCoralState(reefLevel), mainMechanism.presetSubsystems);
    }

    public Command getAlgaeIntakeLevel(){
        return Commands.defer(()->mainMechanism.intakeAlgae((reefLevel+1)/2), mainMechanism.presetSubsystems);
    }

    public Command getAlgaeLevel(){
        return Commands.defer(()->mainMechanism.setAlgaeScoreState(algaeScoreLevel), mainMechanism.presetSubsystems);
    }

    public Command resetOrIntake(){
        return Commands.either(mainMechanism.elevator.reachGoal(0),Commands.either(mainMechanism.intake(),
        mainMechanism.elevator.reachGoal(0), ()->!mainMechanism.intake.hasAlgae()),()->mainMechanism.intake.hasCoral());
    }

    public Command getAutoCoralPosition(){
        return Commands.defer(()->mainMechanism.swerveDrive.toPose(GameData.coralPose(position)
        ,1.2,3, 3),Set.of());
    }

    public void setReefLevel(int value){
        reefLevel = MathUtil.clamp(value, 1, 4);
    }

    public void periodic(){

        Pose2d[] poses = new Pose2d[]{
            GameData.algaePose(1),
            GameData.algaePose(2),
            GameData.algaePose(3),
            GameData.algaePose(4),
            GameData.algaePose(5),
            GameData.algaePose(6)
        };

        Pose2d closest = PoseEX.closestTo(robotPose.get(), poses);

        int place = 1;
        for (int i = 0; i < poses.length; i++){
            if (poses[i] == closest){
                place = i;
                break;
            }
        }
        closestSide = place + 1;

        Pose2d[] feederPoses = new Pose2d[]{GameData.feederPose(1), GameData.feederPose(2)};
        Pose2d closestFeed = PoseEX.closestTo(robotPose.get(), feederPoses);

        int placeFeed = 1;
        for (int i = 0; i < feederPoses.length; i++){
            if (feederPoses[i] == closestFeed){
                placeFeed = i;
                break;
            }
        }
        closestFeeder = placeFeed + 1;

    }
}
