package frc.robot.subsystems.motors.talon;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import frc.robot.subsystems.motors.BaseMotor;

public class TalonMotor implements BaseMotor{
    private TalonFX motor;
    private CANcoder canCoder;
    private double sensorToMechanism;
    private double rotorToSensor;
    private double magnetOffset;

    public TalonMotor(int motorID, int encoderID, double sensorToMechanism, double rotorToSensor, double magnetOffset){
        motor = new TalonFX(motorID,"Canivore");
        canCoder = new CANcoder(encoderID,"Canivore");
        this.sensorToMechanism = sensorToMechanism;
        this.rotorToSensor = rotorToSensor;
        this.magnetOffset = magnetOffset;
        applySlots();
    }

    public TalonMotor(int motorID, double gearing){
        motor = new TalonFX(motorID,"Canivore");
        canCoder = null;
        this.sensorToMechanism = gearing;
        applySlots();
    }

    @Override
    public void set(double x) {
        motor.set(Math.max(Math.min(x,1),-1));
    }

    @Override
    public double getVelocity() {
        if (canCoder == null){
            return motor.getVelocity().getValueAsDouble()/sensorToMechanism;
        }
        return canCoder.getVelocity().getValueAsDouble();
    }

    @Override
    public double getPosition() {
        if (canCoder == null){
            return motor.getPosition().getValueAsDouble()/sensorToMechanism;
        }
        return canCoder.getPosition().getValueAsDouble();
    }

    @Override
    public double getFixedPosition(){
        double fixPos = getPosition()%1;
        if (fixPos > .5){
            fixPos-=1;
        }else if(fixPos < -.5){
            fixPos+=1;
        }
        return fixPos;
    }

    @Override
    public int getMotorID() {
        return motor.getDeviceID();
    }

    @Override
    public int getEncoderID() {
        if (canCoder == null){
            return -1;
        }
        return canCoder.getDeviceID();
    }

    private void applySlots(){
        var slot0Configs = new Slot0Configs();
        slot0Configs.kS = 0.1; // Add 0.1 V output to overcome static friction
        slot0Configs.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
        slot0Configs.kP = 0.11; // An error of 1 rps results in 0.11 V output
        slot0Configs.kI = 0; // no output for integrated error
        slot0Configs.kD = 0; // no output for error derivative

        motor.getConfigurator().apply(slot0Configs);
        TalonFXConfiguration fx_cfg = new TalonFXConfiguration();
        if (canCoder!=null){
            CANcoderConfiguration cc_cfg = new CANcoderConfiguration();
            cc_cfg.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
            cc_cfg.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
            cc_cfg.MagnetSensor.MagnetOffset = magnetOffset;
            canCoder.getConfigurator().apply(cc_cfg);

            fx_cfg.Feedback.FeedbackRemoteSensorID = canCoder.getDeviceID();
            fx_cfg.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
            fx_cfg.Feedback.SensorToMechanismRatio = sensorToMechanism;
            fx_cfg.Feedback.RotorToSensorRatio = rotorToSensor;
        }
        fx_cfg.CurrentLimits.StatorCurrentLimit = 80;
        motor.getConfigurator().apply(fx_cfg);
    }

    @Override
    public BaseMotor setInvert(boolean x) {
        motor.setInverted(x);
        return this;
    }
}
