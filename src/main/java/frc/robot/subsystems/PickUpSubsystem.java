// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


package frc.robot.subsystems;

//import frc.robot.RobotContainer;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PickUpConstants;
 

public class PickUpSubsystem extends SubsystemBase {
  /** Creates a new PickUp. */

  private final WPI_TalonSRX RollerMotor;
  private final WPI_TalonSRX ArmMotor;
  private final DigitalInput UpLimit;
 // private ShuffleboardTab sd_tab;
  //private NetworkTableEntry drivemode;


  public PickUpSubsystem() {

    RollerMotor = new WPI_TalonSRX(PickUpConstants.k_RollerTalon);
    ArmMotor = new WPI_TalonSRX(PickUpConstants.k_ArmTalon);
    UpLimit = new DigitalInput(PickUpConstants.k_PickUpLimit);



  }
/////////////////////////////////////////////////////////////////////////////////
//
// Drive routines
//
/////////////////////////////////////////////////////////////////////////////////
  public void RollerDrive() {
    RollerMotor.set(PickUpConstants.k_RollerSpeed);
  }

  public void RollerOff() {
    RollerMotor.set(0.0);
  }

  public void ArmDriveUp() {
    if (UpLimit.get() != PickUpConstants.k_UpLimitUp) {
      ArmMotor.set(PickUpConstants.k_ArmSpeed);
    }  
  }

  public void ArmDriveDown() {
    ArmMotor.set(-1.0*PickUpConstants.k_ArmSpeed);
  }

  public void ArmDriveOff() {
    ArmMotor.set(0.0);
  }
//////////////////////////////////////////////////////////////////////////
//
// Periodic
//
/////////////////////////////////////////////////////////////////////////
  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }
}
