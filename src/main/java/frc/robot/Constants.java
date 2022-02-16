// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import frc.ConversionHelper;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    // CAN Addresses
    public static final int kMotorFL = 1;
    public static final int kMotorBL = 2;
    public static final int kMotorFR = 3;
    public static final int kMotorBR = 4;
    public static final int kPCM = 52;
    public static final int kshooterSpark = 7;
    public static int kHangerSpark = 8;

    /** The upper limit at which the drive will stop using kDriveReduction */
    public static final double kDriveThreshold = 0.9;
    /** Drive motor speed is multiplied by this value */                                               
    public static final double kDriveReduction = 0.75;
    
    public static final int kGrabbySolenoidIndex = 7;
    public static final int kShootySolenoidIndex = 6;
    public static final int kPickupSolenoidIndex = 5;
    public static final int kSenseyGrabby = 0;
    public static final int kSenseyShooty = 1;
    public static final int kSetPoint = 5000;
    public static final int kreverseSetPoint = -1000;
    public static final int kslowSpeed = 1500;
    public final class DTConsts {
        public static final double kWheelDiameter = 0.50; // Feet 
        public static final double kTrackWidth = 0.0;
        public static final int kTicksPerRevolution = (int)(4096.0*10.71);
        public static final double RAMSETE_B = 2;
        public static final double RAMSETE_ZETA = 0.7;
        public static final double KS = 0;
        public static final double KV = 0;
        public static final double KA = 0;
        public static final double MAX_VELOCITY = 0;
        public static final double MAX_ACCELERATION = 0;
        public static final double kD = 0.0; 
        public static final double kP = 0.0; 
        public static final double kI = 0.0; 
        public static final double kFF = 0.0; 
        public static final int kTimeOut = 10; 
        public static final int kClosedLoopError = 0; 
        public static final int kStatusFrame = 10; 
    }
    public final class LimeLight {
        public static final double kDriveP = 0.26; 
        public static final double kSteerP = 0.03;
        public static final double kDesiredTarget = 15.0;
        public static final double kMaxDrive = 0.5;
        public static final double kMinSpeed = 0.05; 
    }
    public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(DTConsts.kTrackWidth);
    
    
}

