package frc.robot.subsystems;

import org.ironmaple.simulation.IntakeSimulation.IntakeSide;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructSubscriber;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Robot;
import frc.robot.constants.ArmConstants;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.ramp.Ramp;
import frc.robot.util.MapleSimWorld;

public class ClimbMechanism {

    public final Climber climber;

    public final Ramp ramp;

    public static enum ClimbPositions{
        OFF(0,0),
        PREPARED(90,90),
        CLIMBED(0,90);

        private double climberDegrees;
        private double rampDegrees;
        ClimbPositions(double climberDegrees, double rampDegrees){
            this.climberDegrees = climberDegrees;
            this.rampDegrees = rampDegrees;
        }

        public double climberDegrees(){
            return climberDegrees;
        }

        public double rampDegrees(){
            return rampDegrees;
        }
    }

    public ClimbMechanism(Climber climber, Ramp ramp){
        this.climber = climber;
        this.ramp = ramp;
        climber.setDefaultCommand(climber.reachGoalDegrees(ClimbPositions.OFF.climberDegrees()));
        ramp.setDefaultCommand(ramp.reachGoalDegrees(ClimbPositions.OFF.rampDegrees()));
    }

    public Command runState(ClimbPositions position){
        return climber.reachGoalDegrees(position.climberDegrees())
            .alongWith(ramp.reachGoalDegrees(position.rampDegrees()));
    }

    public Command idle(){
        return runState(ClimbPositions.OFF);
    }

    public Command prepare(){
        return runState(ClimbPositions.PREPARED);
    }

    public Command climb(){
        return runState(ClimbPositions.CLIMBED);
    }
}
