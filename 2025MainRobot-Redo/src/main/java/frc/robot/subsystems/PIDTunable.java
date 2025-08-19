package frc.robot.subsystems;

import java.util.function.Consumer;
import frc.robot.util.SendableConsumer;

public class PIDTunable {
    private PIDTunable(){

    }
    
    public static void createPIDSGVAChooser(String id, Consumer<double[]> consumer, double[] initVals){
        SendableConsumer.createSendableChooser(id, new String[]{"P", "I", "D", "S", "G", "V", "A"}, consumer, initVals);
    }

    public static void createPIDChooser(String id, Consumer<double[]> consumer, double[] initVals){
        SendableConsumer.createSendableChooser(id, new String[]{"P", "I", "D"}, consumer, initVals);
    }
}
