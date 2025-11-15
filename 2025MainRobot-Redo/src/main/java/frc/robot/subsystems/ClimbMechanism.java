package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.BaseMechanism.MainStates;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.climb.Climber;
import frc.robot.subsystems.ramp.Ramp;

public class ClimbMechanism {

    public final Climber climber;

    public final Ramp ramp;

    public final Arm arm;

    public static enum ClimbPositions{
        OFF(0,0,.1),
        PREPARED(MainStates.Processor.armRotation,-4,-.5),
        CLIMBED(MainStates.Processor.armRotation,8,-.5);

        private double armRotations;
        private double climberVoltage;
        private double rampRotations;
        ClimbPositions(double armRotations, double climberVoltage, double rampRotations){
            this.climberVoltage = climberVoltage;
            this.armRotations = armRotations;
            this.rampRotations = rampRotations;
        }

        public double climberVoltage(){
            return climberVoltage;
        }

        public double rampRotations(){
            return rampRotations;
        }

        public double armRotations(){
            return armRotations;
        }
    }

    public ClimbMechanism(Arm arm, Climber climber, Ramp ramp){
        this.climber = climber;
        this.ramp = ramp;
        this.arm = arm;
        climber.setDefaultCommand(climber.setVoltage(0));
        ramp.setDefaultCommand(ramp.reachGoal(-.35));
    }

    public Command runState(ClimbPositions position){
        return climber.setVoltage(position.climberVoltage()).repeatedly()
            .alongWith(ramp.reachGoal(position.rampRotations()))
            .alongWith(arm.reachGoal(position.armRotations()));
    }

    public Command idle(){
        return runState(ClimbPositions.OFF);
    }

    public Command prepare(){
        return runState(ClimbPositions.PREPARED).alongWith(Commands.runOnce(()->arm.setDefaultCommand(arm.reachGoal(MainStates.Intake.armRotation))))
            .alongWith(Commands.runOnce(()->ramp.setDefaultCommand(ramp.reachGoal(-.23))));
    }

    public Command climb(){
        return runState(ClimbPositions.CLIMBED).alongWith(Commands.runOnce(()->arm.setDefaultCommand(arm.reachGoal(MainStates.Processor.armRotation))))
            .alongWith(Commands.runOnce(()->ramp.setDefaultCommand(ramp.reachGoal(-.5))));
    }
}
