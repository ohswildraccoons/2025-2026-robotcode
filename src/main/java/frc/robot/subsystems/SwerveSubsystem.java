// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.subsystems.CameraSubsystem;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.io.File;
import java.io.IOException;
import java.util.Optional;
import java.util.function.DoubleSupplier;

import org.photonvision.EstimatedRobotPose;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;
import swervelib.SwerveDrive;
import swervelib.math.SwerveMath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import frc.robot.Constants;
import frc.robot.Constants.SwerveDriveConstants;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.swerve.SwerveSetpoint;
import com.pathplanner.lib.util.swerve.SwerveSetpointGenerator;
 
public class SwerveSubsystem extends SubsystemBase { 
  // bBot is 14 14
  private final SwerveDrive swerveDrive;
  private final CameraSubsystem cameras = CameraSubsystem.getInstance();
  

  /** Creates a new SwerveSubsystem. */
  public SwerveSubsystem() {
    File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(), "swerve");

    try {
      if (RobotBase.isSimulation()) {
        SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
        swerveDrive = new SwerveParser(swerveJsonDirectory)
          .createSwerveDrive(SwerveDriveConstants.maximumSpeed, new Pose2d(0.5,0.5, Rotation2d.kZero)); 
        swerveDrive.setHeadingCorrection(false); // Heading correction should only be used while controlling the robot via angle.
        swerveDrive.setCosineCompensator(false); // Disables cosine compensation for simulations since it causes discrepancies not seen in real life
      } else {
        swerveDrive = new SwerveParser(swerveJsonDirectory)
          .createSwerveDrive(SwerveDriveConstants.maximumSpeed);
      }
    } catch (IOException e) {
      e.printStackTrace();
      System.out.println("Failed to read SwerveDrive JSON files");
      throw new RuntimeException("SwerveDrive initialization failed.");
    } catch (Exception e) {
      e.printStackTrace();
      System.out.println("Unknown error during SwerveDrive initialization");
      throw new RuntimeException("SwerveDrive initialization failed.");
    }

    setupPathPlanner();

  };

  public void setupPathPlanner(){
    RobotConfig config = null;
    try{
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
    }

    // Configure AutoBuilder last
    AutoBuilder.configure(
            this::getPose, // Robot pose supplier
            this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getRobotVelocity, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            (speeds, feedforwards) -> swerveDrive.setChassisSpeeds(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
            new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                    new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
            ),
            config, // The robot configuration
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this // Reference to this subsystem to set requirements
    );
  }

 public ChassisSpeeds getVelocity() {
    return swerveDrive.getFieldVelocity();
  }

  public Pose2d getRobotPose() {
    return swerveDrive.getPose();
  }

  public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY,
      DoubleSupplier angularRotationX) {
    return run(() -> {
      // Make the robot move
      swerveDrive.drive(SwerveMath.scaleTranslation(
          new Translation2d(
              -1 * translationX.getAsDouble() * swerveDrive.getMaximumChassisVelocity(),
              translationY.getAsDouble() * swerveDrive.getMaximumChassisVelocity()),
          0.8),
          Math.pow(angularRotationX.getAsDouble(), 3) * swerveDrive.getMaximumChassisAngularVelocity(), // rotation
          true, // Field relative
          false);
    });
  }

  /**
   * Returns the current pose of the robot.
   *
   * @return The current pose.
   */
  public Pose2d getPose()
  {
    return swerveDrive.getPose();
  }

  public void setPose(Pose2d pose)
  {
    Optional<EstimatedRobotPose> estimation = cameras.getPose();
    if (estimation.isPresent()) {
        Pose3d estimatedPose = estimation.get().estimatedPose;

         swerveDrive.swerveDrivePoseEstimator.addVisionMeasurement(
            estimatedPose.toPose2d(),
             estimation.get().timestampSeconds// jetts a doo doo head
         );
    } 
  }

  /**
   * Resets odometry to the given pose. Gyro angle and module positions do not need to be reset when calling this
   * method.  However, if either gyro angle or module position is reset, this must be called in order for odometry to
   * keep working.
   *
   * @param initialHolonomicPose The pose to set the odometry to
   */
  public void resetOdometry(Pose2d initialHolonomicPose)
  {
    swerveDrive.resetOdometry(initialHolonomicPose);
  }

  /**
   * Gets the current velocity (x, y and omega) of the robot
   *
   * @return A {@link ChassisSpeeds} object of the current velocity
   */
  public ChassisSpeeds getRobotVelocity()
  {
    return swerveDrive.getRobotVelocity();
  }



  /**
   * An example method querying a boolean state of the subsystem (for example, a
   * digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {

    Optional<EstimatedRobotPose> vision = cameras.getPose();

    if (vision.isPresent()) {
      EstimatedRobotPose est = vision.get();
      Pose2d visionPose = est.estimatedPose.toPose2d();

      double error = getPose().getTranslation().getDistance(visionPose.getTranslation());

      if (error < 1.0) {  // Only trust vision if it's not crazy
          swerveDrive.swerveDrivePoseEstimator.addVisionMeasurement(
              visionPose,
              est.timestampSeconds
        );
      }

    }




    SmartDashboard.putNumber("bot x", getPose().getX());
    SmartDashboard.putNumber("bot y", getPose().getY());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
