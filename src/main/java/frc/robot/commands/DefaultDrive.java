// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
//import frc.robot.RobotContainer;
///import frc.robot.Constants.DriveConstants;

public class DefaultDrive extends CommandBase {
  /** Creates a new DefaultDrive. */

  DriveSubsystem m_DriveSubsystem;

  public DefaultDrive(DriveSubsystem subsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_DriveSubsystem = subsystem;
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    //switch ((int)Math.round(SmartDashboard.getNumber("DriveMode",DriveConstants.k_TankDriveMode)))
    /*
    switch (RobotContainer.getDriveMode())
    {
      case DriveConstants.k_ArcadeDriveMode :   m_DriveSubsystem.ArcadeDrive(); break;
      case DriveConstants.k_ArcadeGavinDriveMode : m_DriveSubsystem.ArcadeGavinDrive(); break;
      case DriveConstants.k_MotorTestDriveMode : m_DriveSubsystem.MotorTestDrive(); break;
      default : m_DriveSubsystem.TankDrive();
    }
   */

    m_DriveSubsystem.TankDrive();

  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
