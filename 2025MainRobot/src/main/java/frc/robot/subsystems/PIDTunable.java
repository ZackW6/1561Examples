package frc.robot.subsystems;

import java.security.KeyStore.PrivateKeyEntry;
import java.util.Arrays;
import java.util.function.Consumer;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.Trigger;
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
