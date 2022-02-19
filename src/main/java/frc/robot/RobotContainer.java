// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
//import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
//import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
//import frc.robot.Constants.DriveConstants;
//import frc.robot.Constants.PickUpConstants;
//import edu.wpi.first.networktables.NetworkTable;
//import edu.wpi.first.networktables.NetworkTableEntry;
//import edu.wpi.first.networktables.NetworkTableInstance;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  //private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final DriveSubsystem m_driveSubsystem = new DriveSubsystem();
  //private final PickUpSubsystem m_pickupSubsystem = new PickUpSubsystem();
  private final VisionSubsystem m_visionSubsystem = new VisionSubsystem();

  //private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);
  //private final AutoDrive_c m_autoCommandF = new AutoDrive_c(m_driveSubsystem,DriveConstants.k_AutoDriveSpeed,DriveConstants.k_AutoDriveSpeed);
  //private final AutoDrive_c m_autoCommandB = new AutoDrive_c(m_driveSubsystem,-1*DriveConstants.k_AutoDriveSpeed,-1*DriveConstants.k_AutoDriveSpeed);
  //private final AutoDrive_c m_autoCommandR = new AutoDrive_c(m_driveSubsystem,DriveConstants.k_AutoDriveSpeed,-1*DriveConstants.k_AutoDriveSpeed);
  //private final AutoDrive_c m_autoCommandL = new AutoDrive_c(m_driveSubsystem,-1*DriveConstants.k_AutoDriveSpeed,DriveConstants.k_AutoDriveSpeed);
  //private final Command m_autoCommand12 = m_autoCommandF.withTimeout(1.0);
  //private final Command m_autoCommand13 = m_autoCommandL.withTimeout(1.0);
  private final SequentialCommandGroup m_autoCommand100 = new SequentialCommandGroup(
      new AutoDrive_c(m_driveSubsystem,DriveConstants.k_AutoDriveSpeed,DriveConstants.k_AutoDriveSpeed).withTimeout(1),
      new AutoDrive_c(m_driveSubsystem,DriveConstants.k_AutoDriveSpeed,-1*DriveConstants.k_AutoDriveSpeed).withTimeout(1)
    );

  private final DefaultDrive m_defaultDrive = new DefaultDrive(m_driveSubsystem);
  private final Vision_c m_visionCommand = new Vision_c(m_visionSubsystem);
  private final AutoTurn_c m_autoturnCommand = new AutoTurn_c(m_driveSubsystem, 90.0, 0.5, -0.5);

  private static XboxController m_DriverController = new XboxController(OIConstants.kDriverControllerPort);

  //private static NetworkTableEntry DriveModeEntry;  
  //private static NetworkTableEntry xEntry;

  //SendableChooser<Command> m_chooser = new SendableChooser<>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    m_driveSubsystem.setDefaultCommand(m_defaultDrive);
    m_visionSubsystem.setDefaultCommand(m_visionCommand);

   // m_chooser.addOption("Auto Forward 1", m_autoCommand12);
    //m_chooser.addOption("Auto Left 1", m_autoCommand13);
    //m_chooser.addOption("Auto Forward 1 Left 1", m_autoCommand100);

    //Shuffleboard.getTab("Autonomous").add(m_chooser);

    //SmartDashboard.putData(m_driveSubsystem);

  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    final JoystickButton d_aButton = new JoystickButton(m_DriverController, Button.kA.value);
    //final JoystickButton d_bButton = new JoystickButton(m_DriverController, Button.kB.value);
    //final JoystickButton d_xButton = new JoystickButton(m_DriverController, Button.kX.value);
    //final JoystickButton d_yButton = new JoystickButton(m_DriverController, Button.kY.value);

    d_aButton.whenPressed(m_autoturnCommand);

  }

  public static int getDriveMode() {

    //double drivemode;

    //NetworkTableInstance inst = NetworkTableInstance.getDefault();
    //NetworkTable table = inst.getTable("datatable");

    //DriveModeEntry = table.getEntry("DriveModeEntry");
    //drivemode = DriveModeEntry.getDouble(0); 
   // return (int)Math.round(drivemode);

    return DriveConstants.k_DriveMode;
}
public static double doDeadBand(double input) {
  double m_speedInput = input;
  if (Math.abs(m_speedInput) < OIConstants.kDeadBand) {
    m_speedInput = 0;
  }
  return m_speedInput;
}
  public static double getLeftSpeed() {
      double m_speedLS = m_DriverController.getLeftY();
      if (Math.abs(m_speedLS) < OIConstants.kDeadBand) {
        m_speedLS = 0;
      }
      return m_speedLS;
  }
  public static double getRightSpeed() {
    double m_speedRS = m_DriverController.getRightY();
    if (Math.abs(m_speedRS) < OIConstants.kDeadBand) {
      m_speedRS = 0;
    }
    return m_speedRS;
  }
  public static double getLeftDirection() {
    double m_speedLD = m_DriverController.getLeftX();
    if (Math.abs(m_speedLD) < OIConstants.kDeadBand) {
      m_speedLD = 0;
    }
    return m_speedLD;
}
public static double getRightDirection() {
  double m_speedRD = m_DriverController.getLeftX();
  if (Math.abs(m_speedRD) < OIConstants.kDeadBand) {
    m_speedRD = 0;
  }
  return m_speedRD;
}
public static boolean getAButton() {
  return m_DriverController.getAButton();
}
public static boolean getBButton() {
  return m_DriverController.getBButton();
}
public static boolean getXButton() {
  return m_DriverController.getXButton();
}
public static boolean getYButton() {
  return m_DriverController.getYButton();
}

public static long slowPrint(String text, long i, long cycle) {
  if ((i%cycle) == 0) System.out.println(text);
  return i + 1;
}

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    //return m_autoCommand;
    //return m_autoCommand2.withTimeout(1.0);   // does not work - robot abend
    //return m_autoCommand2;
    //return m_autoCommand12;
    //return m_autoCommand13;
    return m_autoCommand100;

    //return m_chooser.getSelected();
    
  }
}
