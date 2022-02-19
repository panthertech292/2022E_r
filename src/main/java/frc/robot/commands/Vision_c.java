// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.VisionSubsystem;

public class Vision_c extends CommandBase {
  /** Creates a new VisionCommand. */

  private VisionSubsystem m_VisionSubsystem;

  public Vision_c(VisionSubsystem subsystem) {
    m_VisionSubsystem = subsystem;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_VisionSubsystem.DoVision();
    m_VisionSubsystem.initArray(m_VisionSubsystem.getVisionX());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {


    m_VisionSubsystem.DoVision();

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_VisionSubsystem.getArrayAve();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
