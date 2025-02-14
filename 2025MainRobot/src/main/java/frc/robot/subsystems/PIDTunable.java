package frc.robot.subsystems;

import java.security.KeyStore.PrivateKeyEntry;
import java.util.function.Consumer;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class PIDTunable {
    private PIDTunable(){
        
    }
    public static void createPIDChooser(String id, Consumer<double[]> consumer, double[] initPIDvals){
        if (initPIDvals.length < 7) {
            throw new IllegalArgumentException("initPIDvals must have at least 7 elements");
        }

        Notifier notifier;

        NetworkTableEntry entryP;
        NetworkTableEntry entryI;
        NetworkTableEntry entryD;
        NetworkTableEntry entryS;
        NetworkTableEntry entryG;
        NetworkTableEntry entryV;
        NetworkTableEntry entryA;
        NetworkTable table = NetworkTableInstance.getDefault().getTable("SmartDashboard").getSubTable(id);

        entryP = table.getEntry("P");
        entryP.setDefaultDouble(initPIDvals[0]);
        entryI = table.getEntry("I");
        entryI.setDefaultDouble(initPIDvals[1]);
        entryD = table.getEntry("D");
        entryD.setDefaultDouble(initPIDvals[2]);
        entryS = table.getEntry("S");
        entryS.setDefaultDouble(initPIDvals[3]);
        entryG = table.getEntry("G");
        entryG.setDefaultDouble(initPIDvals[4]);
        entryV = table.getEntry("V");
        entryV.setDefaultDouble(initPIDvals[5]);
        entryA = table.getEntry("A");
        entryA.setDefaultDouble(initPIDvals[6]);

        notifier = new Notifier(()->{
            consumer.accept(new double[]{
                entryP.getDouble(initPIDvals[0]),
                entryI.getDouble(initPIDvals[1]),
                entryD.getDouble(initPIDvals[2]),
                entryS.getDouble(initPIDvals[3]),
                entryG.getDouble(initPIDvals[4]),
                entryV.getDouble(initPIDvals[5]),
                entryA.getDouble(initPIDvals[6])
            });
        });
        notifier.startPeriodic(0.02);

        Runtime.getRuntime().addShutdownHook(new Thread(notifier::close));
    }
}
