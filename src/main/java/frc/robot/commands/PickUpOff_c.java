// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PickUpSubsystem;

public class PickUpOff_c extends CommandBase {
  /** Creates a new PickUpOff_c. */

  PickUpSubsystem m_pickupsubsystem;

  public PickUpOff_c(PickUpSubsystem subsystem) {
    m_pickupsubsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_pickupsubsystem.ArmDriveUp();
    m_pickupsubsystem.RollerOff();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_pickupsubsystem.ArmDriveOff();
    m_pickupsubsystem.RollerOff();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
