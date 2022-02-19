// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
//import com.ctre.phoenix.motorcontrol.FeedbackDevice;
//import com.ctre.phoenix.motorcontrol.NeutralMode;
//import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
//import com.revrobotics.CANSparkMax;
//import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;

import frc.robot.RobotContainer;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.networktables.NetworkTable;
//import edu.wpi.first.networktables.NetworkTableEntry;
//import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class DriveSubsystem extends SubsystemBase {
  /** Creates a new DriveSubsystem. */

  private final WPI_TalonSRX FrontLeftMotor;
  private final WPI_TalonSRX FrontRightMotor;
  private final WPI_TalonSRX BackLeftMotor;
  private final WPI_TalonSRX BackRightMotor;

  //private final CANSparkMax FrontLeftMotor;
  //private final CANSparkMax FrontRightMotor;
  //private final CANSparkMax BackLeftMotor;
  //private final CANSparkMax BackRightMotor;

  private final MotorControllerGroup m_LeftSide;
  private final MotorControllerGroup m_RightSide;

  private final DifferentialDrive m_Drive;

  private AHRS ahrs;

  private final Timer m_timer1;
  //private long i;

  //private double v_previous_error;
  //private double v_integral;

  private double v_setPointX;
  private double v_setPointY;

  //private Number test;
  //private int test2;

  //NetworkTableEntry xEntry;
  //NetworkTableEntry DriveModeEntry;

  public DriveSubsystem() {

    FrontLeftMotor = new WPI_TalonSRX(DriveConstants.k_LeftFrontTalon);
    FrontRightMotor = new WPI_TalonSRX(DriveConstants.k_RightFrontTalon);
    BackLeftMotor = new WPI_TalonSRX(DriveConstants.k_LeftBackTalon);
    BackRightMotor = new WPI_TalonSRX(DriveConstants.k_RightBackTalon);

    //FrontLeftMotor = new CANSparkMax(DriveConstants.k_LeftFrontMax,MotorType.kBrushless);
    //FrontRightMotor = new CANSparkMax(DriveConstants.k_RightFrontMax,MotorType.kBrushless);
    //BackLeftMotor = new CANSparkMax(DriveConstants.k_LeftBackMax,MotorType.kBrushless);
    //BackRightMotor = new CANSparkMax(DriveConstants.k_RightBackMax,MotorType.kBrushless);

    //FrontLeftMotor = new CANSparkMax(DriveConstants.k_LeftFrontMaxB,MotorType.kBrushless);
    //FrontRightMotor = new CANSparkMax(DriveConstants.k_RightFrontMaxB,MotorType.kBrushless);

    m_LeftSide = new MotorControllerGroup(FrontLeftMotor,BackLeftMotor);
    m_RightSide = new MotorControllerGroup(FrontRightMotor,BackRightMotor);

    //m_LeftSide = new MotorControllerGroup(FrontLeftMotor);
    //m_RightSide = new MotorControllerGroup(FrontRightMotor);

    m_Drive = new DifferentialDrive(m_LeftSide,m_RightSide);

    //FrontLeftMotor.setNeutralMode(NeutralMode.Brake);
    //FrontRightMotor.setNeutralMode(NeutralMode.Brake);
    //BackRightMotor.setNeutralMode(NeutralMode.Brake);
    //BackLeftMotor.setNeutralMode(NeutralMode.Brake);

    //BackRightMotor.configFactoryDefault();
    //BackLeftMotor.configFactoryDefault();

    //BackLeftMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 1);
    //BackRightMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 1);

    //BackLeftMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
    //BackRightMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);

    //BackLeftMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);
    //BackRightMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);

    try {
      /* Communicate w/navX-MXP via the MXP SPI Bus.                                     */
      /* Alternatively:  I2C.Port.kMXP, SerialPort.Port.kMXP or SerialPort.Port.kUSB     */
      /* See http://navx-mxp.kauailabs.com/guidance/selecting-an-interface/ for details. */
      ahrs = new AHRS(SPI.Port.kMXP); 
    } catch (RuntimeException ex ) {
      DriverStation.reportError("Error instantiating navX-MXP:  " + ex.getMessage(), true);
    }

    m_timer1 = new Timer();
    //i = 0;
    //test = 9;

    //NetworkTableInstance inst = NetworkTableInstance.getDefault();
    //NetworkTable table = inst.getTable("datatable");
    //xEntry = table.getEntry("X");
    //DriveModeEntry = table.getEntry("DriveMode");
    //xEntry.setNumber(0);

    //SmartDashboard.putNumber("DriveMode", DriveConstants.k_TankDriveMode);

  }

/////////////////////////////////////////////////////////////////////
//
// Misc
//
////////////////////////////////////////////////////////////////////
public void timer1_Init() {
   m_timer1.reset();
   m_timer1.start();
}
public boolean timer1_Timeout(double timeout) {
  double m_timeout;
  double m_time;
  m_timeout = timeout;
  m_time = Math.abs(m_timer1.get());
  return (m_time > m_timeout);
}
public void timer1_Stop() {
  m_timer1.stop(); 
  m_timer1.reset();

} 

public double getAngle() {
  return ahrs.getAngle();
}

/////////////////////////////////////////////////////////////////////
//   PID Stuff   ////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////
/* public void Init_PID1() {
  v_integral = 0.0;
  v_previous_error = 0.0;
}

private double PID1(double target, double actual, double feedforward) {
    double m_target;
    double m_actual;
    double m_feedforward;
    double error;
    double P = 0.01;
    //double I = // 0.00812;
    double I = 0.0;
    double D = 0.0;
    double derivative;
    double rcw;

    m_target = target;
    m_actual = actual;
    m_feedforward = feedforward;

    error = m_target - m_actual; // Error = Target - Actual
    v_integral = v_integral + (error * .02); // Integral is increased by the error*time (which is .02 seconds using normal IterativeRobot)
    derivative = (error - v_previous_error) / .02;
    v_previous_error = error;
    rcw = P * error + I * v_integral + D * derivative;
    
    return m_feedforward + rcw;

} */

/////////////////////////////////////////////////////////////////////
//   encoder stuff    ///////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////
/*public double getLeftEncoderValue() {
  return BackLeftMotor.getSelectedSensorPosition();
}
public double getRightEncoderValue(){
  return BackRightMotor.getSelectedSensorPosition();
}
public double getLeftEncoderSpeed() {
  return BackLeftMotor.getSelectedSensorVelocity(0);
}
public double getRightEncoderSpeed() {
  return BackRightMotor.getSelectedSensorVelocity(0);
} */
//////////////////////////////////////////////////////////////////////
//  Drive Modes      /////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
  public void TankDrive() {
    m_Drive.tankDrive((-1*RobotContainer.getLeftSpeed()),RobotContainer.getRightSpeed());
  }

  public void ArcadeDrive() {
    m_Drive.arcadeDrive((RobotContainer.getRightSpeed()), RobotContainer.getRightDirection());
  } 

  public void ArcadeGavinDrive() {
    m_Drive.arcadeDrive((RobotContainer.getRightSpeed()), RobotContainer.getLeftDirection());
  } 

  public void AutoDrive(double setpointx, double setpointy) {

    v_setPointX = setpointx;
    v_setPointY = setpointy;

    m_Drive.tankDrive(v_setPointX, -1* v_setPointY);
  } 
////////////////////////////////////////////////////////////////////////////
// Moter test routines     //////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////
  private double button2speed(boolean button) {
    boolean m_button;
    m_button = button;
    if (m_button) return DriveConstants.k_TestDriveSpeed; else return 0.0;
  }

  public void MotorTestDrive() {
    m_Drive.tankDrive(0.0, 0.0);
    BackLeftMotor.set(button2speed(RobotContainer.getXButton()));
    FrontLeftMotor.set(button2speed(RobotContainer.getYButton()));
    BackRightMotor.set(button2speed(RobotContainer.getAButton()));
    FrontRightMotor.set(button2speed(RobotContainer.getBButton()));
  }

//////////////////////////////////////////////////////////////////////////
// Periodic    ///////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
  //private double x;
  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    //SmartDashboard.putNumber("Left Position", getLeftEncoderValue());
    //SmartDashboard.putNumber("Right Position", getRightEncoderValue());
    //SmartDashboard.putNumber("Left Speed", getLeftEncoderSpeed());
    //SmartDashboard.putNumber("Right Speed", getRightEncoderSpeed());

    SmartDashboard.putNumber("IMU_TotalYaw", ahrs.getAngle());

    //test2 = test.intValue();

    //i = RobotContainer.slowPrint("test = " + i, i, 25L);

    //xEntry.setDouble(x);
    //x += 0.05;
  }
}
