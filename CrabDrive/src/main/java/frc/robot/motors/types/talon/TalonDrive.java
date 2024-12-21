package frc.robot.motors.types.talon;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.motors.baseControl.DriveMotor;

public class TalonDrive implements DriveMotor{
    
    private TalonFX motor;
    private CANcoder canCoder;

    private VelocityVoltage velocityController = new VelocityVoltage(0);

    public TalonDrive(int motorID, int encoderID){
        motor = new TalonFX(motorID);
        canCoder = new CANcoder(encoderID);
        applySlots();
        velocityController.withSlot(0);
    }

    @Override
    public void setVelocity(double velocity) {
        motor.setControl(velocityController.withVelocity(velocity).withAcceleration(1000));
    }

    @Override
    public double getVelocity() {
        return canCoder.getVelocity().getValueAsDouble();
    }

    @Override
    public double getPosition() {
        return canCoder.getPosition().getValueAsDouble();
    }

    private void applySlots(){
        var slot0Configs = new Slot0Configs();
        slot0Configs.kS = 0.1; // Add 0.1 V output to overcome static friction
        slot0Configs.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
        slot0Configs.kP = 0.11; // An error of 1 rps results in 0.11 V output
        slot0Configs.kI = 0; // no output for integrated error
        slot0Configs.kD = 0; // no output for error derivative

        motor.getConfigurator().apply(slot0Configs);

        CANcoderConfiguration cc_cfg = new CANcoderConfiguration();
        cc_cfg.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
        cc_cfg.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        cc_cfg.MagnetSensor.MagnetOffset = 0.4;
        canCoder.getConfigurator().apply(cc_cfg);

        TalonFXConfiguration fx_cfg = new TalonFXConfiguration();
        fx_cfg.Feedback.FeedbackRemoteSensorID = canCoder.getDeviceID();
        fx_cfg.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        fx_cfg.Feedback.SensorToMechanismRatio = 1.0;
        fx_cfg.Feedback.RotorToSensorRatio = 12.8;

        motor.getConfigurator().apply(fx_cfg);
    }
}
