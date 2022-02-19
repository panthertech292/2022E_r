// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final class OIConstants {
        //Controller Mapping
        static final int kOperControllerPort = 1; // USB
        static final int kDriverControllerPort = 0; // USB

        static final double kDeadBand = 0.1;
    }

    public static final class DriveConstants {
        //Motor Mapping
        public static final int k_RightFrontTalon = 1; //1
        public static final int k_RightBackTalon = 3;  //3
        public static final int k_LeftFrontTalon = 2;  //2
        public static final int k_LeftBackTalon = 4;   //4

        public static final int k_RightFrontMax = 1; //1
        public static final int k_RightBackMax = 3;  //3
        public static final int k_LeftFrontMax = 2;  //2
        public static final int k_LeftBackMax = 4;   //4

        public static final int k_LeftFrontMaxB = 7;  //2
        public static final int k_RightFrontMaxB = 8;   //4

        public static final int k_TankDriveMode = 0;
        public static final int k_ArcadeDriveMode = 1;
        public static final int k_ArcadeGavinDriveMode = 2;
        public static final int k_MotorTestDriveMode = 3;
        public static final int k_DriveMode = k_TankDriveMode;

        public static final double k_TestDriveSpeed = 0.6;
        public static final double k_AutoDriveSpeed = 0.5;
    }

    public static final class PickUpConstants {

       public static final int k_RollerTalon = 10;
       public static final int k_ArmTalon = 11;

       public static final int k_PickUpLimit = 1;

       public static final double k_RollerSpeed = 0.2;
       public static final double k_ArmSpeed = 0.5;

       public static final boolean k_UpLimitUp = false;

    }

    public static final class VisionConstants {

        public static final int k_MaxArea = 10;
        public static final boolean k_DoAreaCheck = true;
 
     }

}
