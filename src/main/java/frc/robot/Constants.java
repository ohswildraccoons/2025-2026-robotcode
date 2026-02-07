// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units; 

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
  public static class SwerveDriveConstants{
    public static final double maximumSpeed = Units.feetToMeters(14.5);
  }
  public static class MotorConstants{

    // swerve 1 - 10
    public static final int kLeftFrontDriveMotorPort = 1;
    public static final int kLeftFrontSteerMotorPort = 2;
    public static final int kRightFrontDriveMotorPort = 3;
    public static final int kRightFrontSteerMotorPort = 4;
    public static final int kLeftRearDriveMotorPort = 5;
    public static final int kLeftRearSteerMotorPort = 6;
    public static final int kRightRearDriveMotorPort = 7;
    public static final int kRightRearSteerMotorPort = 8;
   
    // intake 11 - 20
    public static final int kIntakeMotorPort = 11;
    public static final int kIntakeExtendMotorPort = 12;

    // hopper 21 - 30
    public static final int kTravelMotorPort = 21;

    // indexer 31 - 40

    // turret 41-50

    // shooter 51-60
    public static final int kShooterMotorPort = 51;
    public static final int kTurretMotorPort = 52;
    
    
  } 
}
