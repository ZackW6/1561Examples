// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import frc.robot.constants.ShooterConstants;
import frc.robot.simulation.SimMotor;

public class Shooter extends SubsystemBase {

  SimMotor indexer = new SimMotor(ShooterConstants.indexerMotorID);
  SimMotor shooter = new SimMotor(ShooterConstants.shooterMotorID);

  private final NetworkTableInstance inst = NetworkTableInstance.getDefault();
  private final NetworkTable shooterStats = inst.getTable("Shooter");

  private final DoublePublisher shooterSpeed = shooterStats.getDoubleTopic("ShootSpeed").publish();
  private final DoublePublisher indexerSpeed = shooterStats.getDoubleTopic("IndexSpeed").publish();

  /** Creates a new Shooter. */
  public Shooter() {}

  public Command intakeNote(){
    return Commands.run(()->{
      setIndexerSpeed(-1);
      setShooterSpeed(-.5);
    }).finallyDo(()->{
      setIndexerSpeed(0);
      setShooterSpeed(0);
    }).withInterruptBehavior(InterruptionBehavior.kCancelSelf);
  }

  public Command shootNote(){
    return Commands.runOnce(()->setShooterSpeed(1))
    .andThen(Commands.waitSeconds(2))
    .andThen(Commands.runOnce(()->setIndexerSpeed(1)))
    .andThen(Commands.waitSeconds(1)).finallyDo(()->{
      setShooterSpeed(0);
      setIndexerSpeed(0);
    });
  }

  /**
   * -1 to 1
   * @param amount
   */
  public void setIndexerSpeed(double amount){
    indexer.set(amount);
  }

  /**
   * -1 to 1
   * @param amount
   */
  public void setShooterSpeed(double amount){
    shooter.set(amount);
  }

  public double getIndexerSpeed(){
    return indexer.get();
  }

  public double getShooterSpeed(){
    return shooter.get();
  }

  @Override
  public void periodic() {
    indexerSpeed.set(getIndexerSpeed());
    shooterSpeed.set(getShooterSpeed());
  }
}
