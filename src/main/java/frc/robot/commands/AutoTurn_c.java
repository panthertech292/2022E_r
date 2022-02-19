// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class AutoTurn_c extends CommandBase {
  /** Creates a new AutoTurn_c. */
  DriveSubsystem m_Subsystem;
  double m_YawTarget;
  double m_RightSpeed;
  double m_LeftSpeed;
  double baseAngle;
  double currentAngle;
  public AutoTurn_c(DriveSubsystem subsystem, double yawtarget, double rightSpeed, double leftSpeed) {
    m_Subsystem = subsystem;
    m_YawTarget = yawtarget;
    m_LeftSpeed = leftSpeed;
    m_RightSpeed = rightSpeed;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    baseAngle = m_Subsystem.getAngle();
    System.out.println("Init");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currentAngle = m_Subsystem.getAngle();
    m_Subsystem.AutoDrive(m_RightSpeed, m_LeftSpeed);
    System.out.println("running = " + m_RightSpeed + " " + m_LeftSpeed);
    SmartDashboard.putNumber("Base Angle = ", baseAngle);
    SmartDashboard.putNumber("Current Angle = ", currentAngle);
    SmartDashboard.putNumber("Yaw target = ", m_YawTarget);
    SmartDashboard.putNumber("Angle Delta = ", Math.abs(currentAngle - baseAngle));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
   // if (Math.abs((currentAngle - baseAngle)) >= m_YawTarget){ 
   //   System.out.println("Angle Done = " + Math.abs(currentAngle - baseAngle));
   //   return true;
   // }
    return false;
  }
}
