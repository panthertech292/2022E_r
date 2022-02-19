// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class AutoDrive_c extends CommandBase {
  /** Creates a new AuroDrive_c. */
  DriveSubsystem m_drivesubsystem;
  double m_rightSpeed;
  double m_leftSpeed;
  public AutoDrive_c(DriveSubsystem subsystem, double rightspeed, double leftspeed) {
    m_drivesubsystem = subsystem;
    m_rightSpeed = rightspeed;
    m_leftSpeed = leftspeed;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drivesubsystem.AutoDrive(0.0,0.0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drivesubsystem.AutoDrive(m_rightSpeed,m_leftSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivesubsystem.AutoDrive(0.0,0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
