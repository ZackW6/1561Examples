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
import frc.robot.subsystems.climb.Climber;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.ramp.Ramp;
import frc.robot.util.MapleSimWorld;

public class ClimbMechanism {

    public final Climber climber;

    public final Ramp ramp;

    public final Arm arm;

    public static enum ClimbPositions{
        OFF(0,0,.1),
        PREPARED(0,.25,-.25),
        CLIMBED(0,0,-.25);

        private double armRotations;
        private double climberRotations;
        private double rampRotations;
        ClimbPositions(double armRotations, double climberRotations, double rampRotations){
            this.climberRotations = climberRotations;
            this.armRotations = armRotations;
            this.rampRotations = rampRotations;
        }

        public double climberRotations(){
            return climberRotations;
        }

        public double rampRotations(){
            return rampRotations;
        }

        public double armRotations(){
            return rampRotations;
        }
    }

    public ClimbMechanism(Arm arm, Climber climber, Ramp ramp){
        this.climber = climber;
        this.ramp = ramp;
        this.arm = arm;
        climber.setDefaultCommand(climber.reachGoal(ClimbPositions.OFF.climberRotations()));
        ramp.setDefaultCommand(ramp.reachGoal(ClimbPositions.PREPARED.rampRotations()));
    }

    public Command runState(ClimbPositions position){
        return climber.reachGoal(position.climberRotations())
            .alongWith(ramp.reachGoal(position.rampRotations()))
            .alongWith(arm.reachGoal(position.armRotations()));
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
