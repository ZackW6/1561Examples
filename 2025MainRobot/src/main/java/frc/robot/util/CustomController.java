package frc.robot.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.HashMap;

public class CustomController {
    private final GenericHID genericHID;
    private final HashMap<Integer, int[]> buttonMappings = new HashMap<>();

    public CustomController(int port) {
        genericHID = new GenericHID(port);
        initializeMappings();
    }

    private void initializeMappings() {
        buttonMappings.put(1, new int[]{1, 5}); // A
        buttonMappings.put(2, new int[]{2, 5}); // B
        buttonMappings.put(3, new int[]{3, 5}); // C
        buttonMappings.put(4, new int[]{4, 5}); // D
        buttonMappings.put(5, new int[]{1, 6}); // E
        buttonMappings.put(6, new int[]{2, 6}); // F
        buttonMappings.put(7, new int[]{3, 6}); // G
        buttonMappings.put(8, new int[]{4, 6}); // H
        buttonMappings.put(9, new int[]{1, 8}); // I
        buttonMappings.put(10, new int[]{2, 8}); // J
        buttonMappings.put(11, new int[]{3, 8}); // K
        buttonMappings.put(12, new int[]{4, 8}); // L
        buttonMappings.put(13, new int[]{1, 9}); // L1
        buttonMappings.put(14, new int[]{2, 9}); // L2
        buttonMappings.put(15, new int[]{3, 9}); // L3
        buttonMappings.put(16, new int[]{4, 9}); // L4
        buttonMappings.put(17, new int[]{8, 9}); // Processor
        buttonMappings.put(18, new int[]{8, 9}); // Barge
        buttonMappings.put(19, new int[]{1, 10}); // Feeder 1
        buttonMappings.put(20, new int[]{2, 10}); // Feeder 2
        buttonMappings.put(21, new int[]{3, 10}); // Feeder Left
        buttonMappings.put(22, new int[]{4, 10}); // Feeder Right
    }

    private boolean getRawButton(int num) {
        return genericHID.getRawButton(MathUtil.clamp(num, 1, 11));
    }

    public Trigger rawButtonPressed(int num) {
        return new Trigger(() -> getRawButton(num));
    }

    public boolean getFixedButton(int num) {
        return buttonMappings.containsKey(num) && getRawButton(buttonMappings.get(num)[0]) && getRawButton(buttonMappings.get(num)[1]);
    }

    public Trigger fixedButtonPressed(int num) {
        return new Trigger(() -> getFixedButton(num));
    }
}