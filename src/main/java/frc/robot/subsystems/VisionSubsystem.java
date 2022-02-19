// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
//import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
//import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
//import frc.robot.Constants.VisionConstants;



public class VisionSubsystem extends SubsystemBase {
  /** Creates a new VisionSubsystem. */

  private NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  private NetworkTableEntry tx = table.getEntry("tx");
  private NetworkTableEntry ty = table.getEntry("ty");
  private NetworkTableEntry ta = table.getEntry("ta");
  private NetworkTableEntry tv = table.getEntry("tv");

  private double x;
  private double y;
  private double area;
  //private double ave;
  private double valid;

  private double[] valuesArray = new double[26];
  private int arrayLen = 25;
  private int i;
  private int arrayIndex;


  public VisionSubsystem() {

  }

  public void DoVision() {
    x = tx.getDouble(0.0);
    y = ty.getDouble(0.0);
    area = ta.getDouble(0.0);
    valid = tv.getDouble(0.0);
 
    if (valid > 0) {
      putArrayEntry(x);
    } 

    //ave = getArrayAve();
  }

  public double getVisionX() {
    return x;
  }

  public double getVisionY() {
    return y;
  }  
  
  public double getVisionA() {
    return area;
  }

  public double getVisionV() {
    return valid;
  }

  public void initArray(double init) {
    for (i = 1; i <= arrayLen; i++) {
       valuesArray[i] = init;
    }
    arrayIndex = 1;
  }  

public double getArrayAve() {
  double valuesArraySum = 0;
  for (i = 1; i <= arrayLen; i++) {
     valuesArraySum = valuesArraySum + valuesArray[i];
  }
  return valuesArraySum / arrayLen;
}

public void putArrayEntry(double val) {
  valuesArray[arrayIndex] = val;
  arrayIndex++;
  if (arrayIndex > arrayLen) arrayIndex = 1;
}


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
        
    //post to smart dashboard periodically
    //SmartDashboard.putNumber("LimelightX", x);
    //SmartDashboard.putNumber("LimelightY", y);
    //SmartDashboard.putNumber("LimelightArea", area);
    //SmartDashboard.putNumber("LimelightAve", ave);
    //SmartDashboard.putNumber("LimelightValid", valid);





  }
}
