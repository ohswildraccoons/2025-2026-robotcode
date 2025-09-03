// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.io.File;
import java.io.IOException;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.Filesystem;
import swervelib.parser.SwerveParser;
import swervelib.SwerveDrive;
import swervelib.math.SwerveMath;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import frc.robot.Constants;
import frc.robot.Constants.SwerveDriveConstants;

public class SwerveSubsystem extends SubsystemBase {
private final SwerveDrive swerveDrive;

  /** Creates a new SwerveSubsystem. */
  public SwerveSubsystem() {
    File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(), "swerve"); 
    SwerveDrive tempDrive = null;
  
    try {
      tempDrive = new SwerveParser(swerveJsonDirectory)
          .createSwerveDrive(SwerveDriveConstants.maximumSpeed);
    } catch (IOException e) {
      e.printStackTrace();
    }
  
    if (tempDrive != null) {
      swerveDrive = tempDrive;
    } else {
      System.out.println("Failed to create SwerveDrive");
      throw new RuntimeException("SwerveDrive initialization failed.");
    }

    

  }
  ;

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    


    return runOnce(
        () -> {
          /* one-time action goes here */
        });

        
  }


  // public void drive(Translation2d translation, double rotation, boolean fieldRelative)
  // {
  //   swerveDrive.drive(translation,
  //                     rotation,
  //                     fieldRelative,
  //                     false); // Open loop is disabled since it shouldn't be used most of the time.
  // }



   public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier angularRotationX)
  {
    return run(() -> {
      // Make the robot move
      swerveDrive.drive(SwerveMath.scaleTranslation(new Translation2d(
                            translationX.getAsDouble() * swerveDrive.getMaximumChassisVelocity(),
                            translationY.getAsDouble() * swerveDrive.getMaximumChassisVelocity()), 0.8),
                        Math.pow(angularRotationX.getAsDouble(), 3) * swerveDrive.getMaximumChassisAngularVelocity(),
                        true,
                        false);
    });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
