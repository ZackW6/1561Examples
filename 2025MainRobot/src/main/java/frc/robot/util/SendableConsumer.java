package frc.robot.util;

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

public class SendableConsumer {
    private SendableConsumer(){

    }
    public static void createSendableChooser(String id, Consumer<Double> consumer, double initVal){
        Notifier notifier;

        NetworkTableEntry entry;
        NetworkTable table = NetworkTableInstance.getDefault().getTable("SmartDashboard");
        
        entry = table.getEntry(id);


        notifier = new Notifier(()->{
            consumer.accept(entry.getDouble(initVal));
        });
        notifier.startPeriodic(0.05);

        Runtime.getRuntime().addShutdownHook(new Thread(notifier::close));
    }

    public static void createSendableChooser(String id, String[] items, Consumer<double[]> consumers, double[] initVals){
        Notifier notifier;

        NetworkTableEntry[] entry = new NetworkTableEntry[items.length];
        NetworkTable table = NetworkTableInstance.getDefault().getTable("SmartDashboard").getSubTable(id);
        
        for (int i = 0; i < items.length; i++){
            entry[i] = table.getEntry(items[i]);
        }

        notifier = new Notifier(()->{
            double[] values = new double[entry.length];
            for (int i = 0; i < entry.length; i++){
                values[i] = entry[i].getDouble(initVals[i]);
            }
            consumers.accept(values);
        });
        notifier.startPeriodic(0.05);

        Runtime.getRuntime().addShutdownHook(new Thread(notifier::close));
    }

    public static void createSendableChooser(String id, String[] items, Consumer<Double>[] consumer, double[] initVals){
        Notifier notifier;

        NetworkTableEntry[] entry = new NetworkTableEntry[items.length];
        NetworkTable table = NetworkTableInstance.getDefault().getTable("SmartDashboard").getSubTable(id);
        
        for (int i = 0; i < items.length; i++){
            entry[i] = table.getEntry(items[i]);
        }

        notifier = new Notifier(()->{
            for (int i = 0; i < entry.length; i++){
                consumer[i].accept(entry[i].getDouble(initVals[i]));
            }
        });
        notifier.startPeriodic(0.05);

        Runtime.getRuntime().addShutdownHook(new Thread(notifier::close));
    }

}
