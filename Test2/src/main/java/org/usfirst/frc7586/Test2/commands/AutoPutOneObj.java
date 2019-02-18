/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc7586.Test2.commands;

import com.ctre.phoenix.motion.TrajectoryPoint;

import org.usfirst.frc7586.Test2.Robot;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.command.Command;
import jaci.pathfinder.*;
import jaci.pathfinder.followers.*;
import jaci.pathfinder.modifiers.*;
import jaci.jniloader.*;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.PathfinderFRC;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.followers.EncoderFollower;

public class AutoPutOneObj extends Command {
  private static final int k_ticks_per_rev = 1024;
  private static final double k_wheel_diameter = 4.0 / 12.0;
  private static final double k_max_velocity = 10;

  private static final double kp = 1.0;
  private static final double ki = 0;
  private static final double kd = 0;
  private static final double kv = 1 / k_max_velocity;
  private static final double ka = 0;

  private EncoderFollower m_left_follower;
  private EncoderFollower m_right_follower;
  private Notifier m_follower_notifier;

  private static final String kPathName = "getOneBall_from_Blue1_to_BlueLeft3";

  public AutoPutOneObj() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.drivetrain);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Trajectory left_trajectory = PathfinderFRC.getTrajectory(kPathName + ".right");
    Trajectory right_trajectory = PathfinderFRC.getTrajectory(kPathName + ".left");

    m_left_follower = new EncoderFollower(left_trajectory);
    m_right_follower = new EncoderFollower(right_trajectory);

    m_left_follower.configureEncoder(Robot.drivetrain.getLeftEncoderPosition(), k_ticks_per_rev, k_wheel_diameter);
    m_right_follower.configureEncoder(Robot.drivetrain.getRightEncoderPosition(), k_ticks_per_rev, k_wheel_diameter);

    m_left_follower.configurePIDVA(kp, ki, kd, kv, ka);
    m_right_follower.configurePIDVA(kp, ki, kd, kv, ka);

    m_follower_notifier = new Notifier(this::followPath);
    m_follower_notifier.startPeriodic(left_trajectory.get(0).dt);

  }

  private void followPath() {
    if (m_left_follower.isFinished() || m_right_follower.isFinished()) {
      m_follower_notifier.stop();
    } else {
      double left_speed = m_left_follower.calculate(Robot.drivetrain.getLeftEncoderPosition());
      double right_speed = m_right_follower.calculate(Robot.drivetrain.getRightEncoderPosition());
      double heading = Robot.drivetrain.getAngle();
      double desired_heading = Pathfinder.r2d(m_left_follower.getHeading());
      double heading_difference = Pathfinder.boundHalfDegrees(desired_heading - heading);
      double turn = 0.8 * (-1.0 / 80.0) * heading_difference;
      Robot.drivetrain.setSpeedControllerGroupLeftSpeed(left_speed - turn);
      Robot.drivetrain.setSpeedControllerGroupRightSpeed(right_speed - turn);
    }
  }

  /**
   * This function is called periodically during autonomous.
   */
  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    m_follower_notifier.stop();
    Robot.drivetrain.setSpeedControllerGroupLeftSpeed(0);
    Robot.drivetrain.setSpeedControllerGroupRightSpeed(0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
