package frc.robot.util;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Optional;
import java.util.Scanner;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj2.command.button.Trigger;


public class CustomController {
    private String currentLoad = "{}";

    private ArrayList<Integer> allInputs = new ArrayList<>();

    private final Notifier checkButtons;
    private final GenericHID genericHID;
    private final HashMap<Integer, int[]> buttonMappings = new HashMap<>();

    private final String portName = "COM5";

    private boolean connected = false;

    public CustomController(int port) {
        genericHID = new GenericHID(port);
        checkButtons = new Notifier(this::checkFiles);
        checkButtons.startPeriodic(.01);
        Runtime.getRuntime().addShutdownHook(new Thread(checkButtons::close));
        initializeMappings();
    }

    private void checkFiles(){
        System.out.println(connected);
        allInputs.clear();
        try (FileInputStream serialInput = new java.io.FileInputStream(portName)) {
            int data;
            StringBuilder receivedData = new StringBuilder();

            while ((char)(data = serialInput.read()) != 'E') { // Read until END
                receivedData.append((char) data);
            }

            currentLoad = receivedData.toString();
            connected = true;
        } catch (Exception e) {
            System.out.println("BAD PORT");
            connected = false;
            return;
        }
        try {
            String allNums = currentLoad.substring(currentLoad.indexOf("{"),currentLoad.indexOf("}")).toString()
                .replace(",","")
                .replace("{","")
                .replace("}","");

            for (char c : allNums.toCharArray()){
                allInputs.add(Integer.parseInt(String.valueOf(c)));
            }

        } catch (Exception e) {
            // TODO: handle exception
        }
        System.out.println(allInputs);
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
        return allInputs.contains(MathUtil.clamp(num - 1,0,9));
        // return genericHID.getRawButton(MathUtil.clamp(num, 1, 11));
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

    public boolean connected(){
        return connected;
    }

    // public static void main(String[] args) {
    //     try {
    //         // Open the serial port as a file (Windows: COMx, Linux/macOS: /dev/ttyUSBx or /dev/ttyACMx)
    //         FileInputStream serialInput = new java.io.FileInputStream("COM5"); // Change to your actual port

    //         while (true) {
    //             if (serialInput.read() > 0) {  // Check if data is available
    //                 int data = serialInput.read();  // Read one byte
    //                 System.out.print((char) data);  // Print as a character (modify as needed)
    //             }
    //         }
    //     } catch (IOException e) {
    //         e.printStackTrace();
    //     }
        
    // }
}