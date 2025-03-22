package frc.robot;

import java.util.Set;
import java.util.function.BooleanSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.FactoryCommands;
import frc.robot.constants.GameData;
import frc.robot.util.CustomController;
import frc.robot.util.SendableConsumer;

public class OptionController {

    private int reefLevel = 4;
    private int reefPosition = 1;

    private int defaultFeeder = 1;

    private int feederPosition = 2;

    private int algaeScoreLevel = 1;

    private final FactoryCommands factoryCommands;

    private final BooleanSupplier hasCoral;

    private final BooleanSupplier hasAlgae;

    public OptionController(CustomController controller, FactoryCommands factoryCommands, BooleanSupplier hasCoral, BooleanSupplier hasAlgae){

        this.factoryCommands = factoryCommands;
        this.hasCoral = hasCoral;
        this.hasAlgae = hasAlgae;

        controller.fixedButtonPressed(1).onTrue(Commands.runOnce(()->{
            //Intended to be A scoring
            reefPosition = 1;
        }));
        controller.fixedButtonPressed(2).onTrue(Commands.runOnce(()->{
            //Intended to be B scoring
            reefPosition = 2;
        }));
        controller.fixedButtonPressed(3).onTrue(Commands.runOnce(()->{
            //Intended to be C scoring
            reefPosition = 3;
        }));
        controller.fixedButtonPressed(4).onTrue(Commands.runOnce(()->{
            //Intended to be D scoring
            reefPosition = 4;
        }));
        controller.fixedButtonPressed(5).onTrue(Commands.runOnce(()->{
            //Intended to be E scoring
            reefPosition = 5;
        }));
        controller.fixedButtonPressed(6).onTrue(Commands.runOnce(()->{
            //Intended to be F scoring
            reefPosition = 6;
        }));
        controller.fixedButtonPressed(7).onTrue(Commands.runOnce(()->{
            //Intended to be G scoring
            reefPosition = 7;
        }));
        controller.fixedButtonPressed(8).onTrue(Commands.runOnce(()->{
            //Intended to be H scoring
            reefPosition = 8;
        }));
        controller.fixedButtonPressed(9).onTrue(Commands.runOnce(()->{
            //Intended to be I scoring
            reefPosition = 9;
        }));
        controller.fixedButtonPressed(10).onTrue(Commands.runOnce(()->{
            //Intended to be J scoring
            reefPosition = 10;
        }));
        controller.fixedButtonPressed(11).onTrue(Commands.runOnce(()->{
            //Intended to be K scoring
            reefPosition = 11;
        }));
        controller.fixedButtonPressed(12).onTrue(Commands.runOnce(()->{
            //Intended to be L scoring
            reefPosition = 12;
        }));
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
        // controller.fixedButtonPressed(17).onTrue(Commands.runOnce(()->{
        //     //Intended to be processor scoring
        //     algaeScoreLevel = 1;
        // }));
        // controller.fixedButtonPressed(18).onTrue(Commands.runOnce(()->{
        //     //Intended to be processor scoring
        //     algaeScoreLevel = 2;
        // }));
        controller.fixedButtonPressed(19).onTrue(Commands.runOnce(()->{
            //Intended to be feeder 1 intake
            defaultFeeder = 1;
        }));
        controller.fixedButtonPressed(20).onTrue(Commands.runOnce(()->{
            //Intended to be feeder 2 intake
            defaultFeeder = 2;
        }));

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
        ,hasAlgae),factoryCommands.mainSubsytems);
    }

    /**
     * scores coral automatically if it has coral, if it does not it goes and grabs an algae, if it had an algae, it scores.
     * @return
     */
    public Command getAutoScore(){
        return Commands.defer(()->Commands.either(coralTillInterruptSwap()
            , Commands.either(algaeTillInterruptSwap()
                , algaeIntakeTillInterruptSwap()
                , hasAlgae)
            ,hasCoral),factoryCommands.mainSubsytems);
    }

    /**
     * 
     * @return
     */
    public Command getAutoCoral(){
        return Commands.defer(()->Commands.either(algaeTillInterruptSwap()
        ,Commands.either(coralTillInterruptSwap()
            , coralIntakeTillInterruptSwap()
            , hasCoral)
        ,hasAlgae),factoryCommands.mainSubsytems).andThen(Commands.defer(()->getAutoCoral(), factoryCommands.mainSubsytems));
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
            ,hasCoral),factoryCommands.mainSubsytems).andThen(Commands.defer(()->getAutoAlgae(), factoryCommands.mainSubsytems));
    }

    private Command coralTillInterruptSwap(){
        return Commands.defer(()->{
            int initReefPosition = reefPosition;
            int initReefLevel = reefLevel;
            return factoryCommands.autoScoreCoral(initReefPosition, initReefLevel)
                .until(()->initReefPosition != reefPosition || (initReefLevel != reefLevel && initReefLevel != 4))
                .andThen(coralTillInterruptSwap()).unless(()->!hasCoral.getAsBoolean());
        },Set.of());
    }

    private Command coralIntakeTillInterruptSwap(){
        return Commands.defer(()->{
            int initFeederDefault = defaultFeeder;
            int initFeederPosition = feederPosition;
            return factoryCommands.autoIntakeCoral(initFeederDefault, (initFeederPosition-2)*GameData.optionalFeederRightOffset)
                .until(()->initFeederDefault != defaultFeeder || initFeederPosition != feederPosition)
                .andThen(coralIntakeTillInterruptSwap()).unless(hasCoral);
        },Set.of());
    }

    private Command algaeTillInterruptSwap(){
        return Commands.defer(()->{
            int initAlgaeLevel = algaeScoreLevel;
            return factoryCommands.autoScoreAlgae(initAlgaeLevel)
                .until(()->initAlgaeLevel != algaeScoreLevel)
                .andThen(algaeTillInterruptSwap()).unless(()->!hasAlgae.getAsBoolean());
        },Set.of());
    }

    private Command algaeIntakeTillInterruptSwap(){
        return Commands.defer(()->{
            int initReefPosition = reefPosition;
            return factoryCommands.autoIntakeAlgae((reefPosition+1)/2)
                .until(()->initReefPosition != reefPosition)
                .andThen(algaeIntakeTillInterruptSwap()).unless(hasAlgae);
        },Set.of());
    }

    public Command getScoreLevel(){
        return Commands.defer(()->factoryCommands.scoringMechanism.presetCoral(reefLevel), factoryCommands.presetSubsytems);
    }

    public Command getAlgaeIntakeLevel(){
        return Commands.defer(()->factoryCommands.scoringMechanism.intakeAlgae((reefLevel+1)/2), factoryCommands.presetSubsytems);
    }

    public Command getAlgaeLevel(){
        return Commands.defer(()->factoryCommands.scoringMechanism.presetAlgae(1), factoryCommands.presetSubsytems);
    }

    public Command resetOrIntake(){
        return Commands.either(factoryCommands.scoringMechanism.elevator.reachGoal(0),Commands.either(factoryCommands.scoringMechanism.intake(),
        factoryCommands.scoringMechanism.elevator.reachGoal(0), ()->!factoryCommands.scoringMechanism.intake.hasAlgae()),()->factoryCommands.scoringMechanism.intake.hasCoral());
    }

    public Command getAutoCoralPosition(){
        return Commands.defer(()->factoryCommands.toPose(GameData.coralPose(reefPosition, DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red)
        ,1.2,3),Set.of());
    }

    public void setReefLevel(int value){
        reefLevel = MathUtil.clamp(value, 1, 4);
    }
}
