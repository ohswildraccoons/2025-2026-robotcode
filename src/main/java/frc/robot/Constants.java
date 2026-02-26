// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
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
    public static final int kIntakeTravelerMotorPort = 11;
    public static final int kIntakeExtendMotorPort = 12;
    public static final int kIntakeMotorPort = 13;

    // hopper 21 - 30
    public static final int kTravelMotorPort = 21;

    // indexer 31 - 40

    // turret 41-50

    // shooter 51-60
    public static final int kShooterMotorPort = 51;
    public static final int kTurretMotorPort = 52;
    
    
  } 

  public static class FieldConstants{
    public static final Pose3d middleField = new Pose3d( 8.3, 3.8, 0, new Rotation3d(0 ,0 ,0));

    public static final Pose3d blueHub = new Pose3d(4.6, 4, 0, new Rotation3d(0 ,0 ,0));
    public static final Pose3d redHub = new Pose3d(12, 4, 0, new Rotation3d(0 ,0 , 0));

    public static final Pose3d blueRightDeposit= new Pose3d(2,1.5, 0, new Rotation3d(0 ,0 ,0));
    public static final Pose3d blueLeftDeposit = new Pose3d(2, 6.5, 0, new Rotation3d(0 ,0 ,0));
    public static final Pose3d redLeftDeposit = new Pose3d(14.6, 1.5, 0, new Rotation3d(0 ,0 ,0));
    public static final Pose3d redRightDeposit = new Pose3d(14.6, 6.5, 0, new Rotation3d(0 ,0 ,0));

    public enum FieldSection{
      BLUE_LEFT, BLUE_RIGHT, BLUE_MIDDLE, BLUE_HUB,  
      RED_LEFT, RED_RIGHT,  RED_MIDDLE, RED_HUB,
      UNKNOWN
    }
  }
  
  public static final class CameraConstants{

      public static final double topFrontX = Units.inchesToMeters(3.0);
      public static final double topFrontY = Units.inchesToMeters(13.0);
      public static final double topFrontZ = Units.inchesToMeters(32.5);
      public static final double topFrontRoll = Units.degreesToRadians(0.0);
      public static final double topFrontPitch = Units.degreesToRadians(-40.0);
      public static final double topFrontYaw = Units.degreesToRadians(0.0);

      public static final double bottomFrontX = Units.inchesToMeters(8.5);
      public static final double bottomFrontY = Units.inchesToMeters(1.5);
      public static final double bottomFrontZ = Units.inchesToMeters(11.0);
      public static final double bottomFrontRoll = Units.degreesToRadians(0.0);
      public static final double bottomFrontPitch = Units.degreesToRadians(0.0);     
      public static final double bottomFrontYaw = Units.degreesToRadians(0.0);
 
      public static final double leftBackX = Units.inchesToMeters(-13.5);
      public static final double leftBackY = Units.inchesToMeters(12.875);
      public static final double leftBackZ = Units.inchesToMeters(23.0);
      public static final double leftBackRoll = Units.degreesToRadians(0.0);
      public static final double leftBackPitch = Units.degreesToRadians(0.0);
      public static final double leftBackYaw = Units.degreesToRadians(168.69);
          
      public static final double rightBackX =  Units.inchesToMeters(-13.5);
      public static final double rightBackY =  Units.inchesToMeters(-12.875);
      public static final double rightBackZ = Units.inchesToMeters(23.0);
      public static final double rightBackRoll = Units.degreesToRadians(0.0);
      public static final double rightBackPitch = Units.degreesToRadians(0.0);
      public static final double rightBackYaw = Units.degreesToRadians(-168.69);
    }


  

  
}
