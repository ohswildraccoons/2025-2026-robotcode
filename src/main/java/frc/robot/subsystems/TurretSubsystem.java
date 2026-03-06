// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Rotation;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import java.util.Optional;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.revrobotics.spark.SparkLowLevel.MotorType;

// import dev.doglog.DogLog;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.SmartMechanism;
import yams.mechanisms.config.ArmConfig;
import yams.mechanisms.config.PivotConfig;
import yams.mechanisms.positional.Arm;
import yams.mechanisms.positional.Pivot;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;

import frc.robot.Constants;
import frc.robot.Constants.MotorConstants;
import frc.robot.Constants.TurretConstants;
import frc.robot.Constants.FieldConstants;


public class TurretSubsystem extends SubsystemBase{ 

  double angle = 0;
  double botRelativeXPos;
  double botRelativeYPos;
  Pose3d turretFieldRelativePose;
  boolean isManualSetpointTargeting = false;
  public enum TurretSide {
    LEFT,
    RIGHT
  }
  TurretSide turretSide;
  Supplier<Pose3d> target;

    
  private SmartMotorControllerConfig smcConfig = new SmartMotorControllerConfig(this)
  .withControlMode(ControlMode.CLOSED_LOOP)
  .withClosedLoopController(4.0, 0.0, 0.0)
  //.withClosedLoopController(4, 0, 0, DegreesPerSecond.of(180), DegreesPerSecondPerSecond.of(90)) Profiled PID breaks the thing?! - hs 20JAN
//  .withSimClosedLoopController(4.0, 0, 0, DegreesPerSecond.of(180), DegreesPerSecondPerSecond.of(90))
  // Configure Motor and Mechanism properties
  .withGearing(new MechanismGearing(GearBox.fromReductionStages(10, 1)))
  .withIdleMode(MotorMode.BRAKE)
  .withMotorInverted(false)
  // Setup Telemetry
  .withTelemetry("TurretMotor", TelemetryVerbosity.HIGH)
  // Power Optimization
  .withStatorCurrentLimit(Amps.of(40))
  .withClosedLoopRampRate(Seconds.of(0.25))
  .withOpenLoopRampRate(Seconds.of(0.25));

  
  // Vendor motor controller object
  private SparkFlex spark = new SparkFlex(Constants.MotorConstants.kTurretMotorPort, MotorType.kBrushless);

  // Create our SmartMotorController from our Spark and config with the NEO.
  private SmartMotorController sparkSmartMotorController = new SparkWrapper(spark, DCMotor.getNEO(1), smcConfig);

  PivotConfig                m_config         = new PivotConfig(sparkSmartMotorController)
      .withStartingPosition(Degrees.of(0)) // Starting position of the Pivot
      .withWrapping(Degrees.of(0), Degrees.of(360)) // Wrapping enabled bc the pivot can spin infinitely
      .withHardLimit(Degrees.of(-7200000.0), Degrees.of(720000000)) // Hard limit bc wiring prevents infinite spinning
      .withTelemetry("TurretPivot", TelemetryVerbosity.HIGH) // Telemetry
      .withMOI(Feet.of(0.25), Pounds.of(4)); // MOI Calculation
     

  // Arm Mechanism
  private Pivot turrePivot = new Pivot(m_config);

  /**
   * Set the angle of the arm.
   * @param angle Angle to go to.
   */
  public Command setAngle(Supplier<Angle> angle) {
    return turrePivot.setAngle(() -> {
      return angle.get();
    });
  }

/*
 * Get the angle of the Turret.
 *  @return angle
 */
public Angle getAngle(){return turrePivot.getAngle();}

  /*
  * Gets the field relative position of the turret, by taking the bot rotation, position, and the bot relative pos
  * @ BotRelative Pos
  */
  public Pose3d getFieldPos(Pose3d botPose){

    double botX = botPose.getX();
    double botY = botPose.getY();
    Rotation2d rotation = botPose.getRotation().toRotation2d();
    Rotation2d turretAngle = new Rotation2d(turrePivot.getAngle());
    Rotation3d turretRotation = new Rotation3d(0, 0, turretAngle.getRadians());

    double fieldRelativeXPos = Math.cos(rotation.getRadians()) * botRelativeXPos - Math.sin(rotation.getRadians()) * botRelativeYPos + botX;
    double fieldRelativeYPos = Math.sin(rotation.getRadians()) * botRelativeXPos + Math.cos(rotation.getRadians()) * botRelativeYPos + botY;
    
    return new Pose3d(fieldRelativeXPos, fieldRelativeYPos, 0, turretRotation);
  }

    /**
     * Move the arm up and down.
     * @param dutycycle [-1, 1] speed to set the arm too.
     */
    public Command set(double dutycycle) { return turrePivot.set(dutycycle);}

    /**
     * Run sysId on the {@link Arm}
     */
    public Command sysId() { return turrePivot.sysId(Volts.of(7), Volts.of(2).per(Second), Seconds.of(4));}

    public Command joystickTurret(DoubleSupplier x, DoubleSupplier y) {
      return turrePivot.setAngle(() -> {
          angle = Math.atan2(y.getAsDouble() * -1, x.getAsDouble());
          double angleDeg = Math.toDegrees(angle) * -1;
          SmartDashboard.putNumber("TEST", angleDeg);
          // turrePivot.setAngle(Degrees.of(angleDeg));
          return Degrees.of(angleDeg);
      });
  }

    public Command targetPose(Supplier<Pose3d> robotPose, Supplier<Pose3d> targetPose, SwerveSubsystem swerveSubsystem) {
      return turrePivot.setAngle(() -> {
        turretFieldRelativePose = getFieldPos(robotPose.get());
          SmartDashboard.putString("Field Section", getFieldSection(robotPose.get()).name());
        Pose3d virtualTargetPose = ghostTargetPose(targetPose.get(), turretFieldRelativePose, swerveSubsystem);
        double deltaX = virtualTargetPose.getX() - turretFieldRelativePose.getX();
        double deltaY = virtualTargetPose.getY() - turretFieldRelativePose.getY();
        double angle = Math.atan2(deltaY, deltaX);
        double angleDeg = adjustAngleToFieldRelative(robotPose, () -> Math.toDegrees(angle));
        SmartDashboard.putNumber("Turret Target Angle", angleDeg);
        

        return Degrees.of(angleDeg);
      });
    }

    public double adjustAngleToFieldRelative(Supplier<Pose3d> robotPose, Supplier<Double> angle) {
        double fieldRelativeAngle = angle.get() - robotPose.get().getRotation().getZ();
        return fieldRelativeAngle;
    }

    public Command autoFindTargetPose(Supplier<Pose3d> robotPose, SwerveSubsystem swerveSubsystem) {

        Supplier<Pose3d> targetPose = () -> {
            switch (getFieldSection(robotPose.get())) {
                case BLUE_HUB:
                    return FieldConstants.blueHub;
                case RED_HUB:
                    return FieldConstants.redHub;
                case BLUE_LEFT:
                    return FieldConstants.blueLeftDeposit;
                case BLUE_RIGHT:
                    return FieldConstants.blueRightDeposit;
                case RED_LEFT:
                    return FieldConstants.redLeftDeposit;
                case RED_RIGHT:
                    return FieldConstants.redRightDeposit;
                case BLUE_MIDDLE:
                    if (turretSide == TurretSide.LEFT) {
                        return FieldConstants.blueLeftDeposit;
                    } else {
                        return FieldConstants.blueRightDeposit;
                    }
                default:
                    return FieldConstants.middleField;
            }
        };
        return targetPose(robotPose, targetPose, swerveSubsystem);
    }

    public Command targettingCommand(Supplier<Pose3d> robotPose, SwerveSubsystem swerveSubsystem) {
        if (isManualSetpointTargeting == false) {
            return autoFindTargetPose(robotPose, swerveSubsystem);
        } else {
            return targetPose(robotPose, target, swerveSubsystem);
        }
        
    }

    public Command setAutoTargettingOff() {
        return runOnce(() -> {
            isManualSetpointTargeting = false;
        });
    }

    public Command setManualTarget(Supplier<Pose3d> target) {
        return runOnce(() -> {
            this.target = target;
            isManualSetpointTargeting = true;
        });
    }

    

    public FieldConstants.FieldSection getFieldSection(Pose3d robotPose) {
        Pose3d turretFieldRelativePose = getFieldPos(robotPose);
        double x = turretFieldRelativePose.getX();
        double y = turretFieldRelativePose.getY();

        

        if (x < 4) {
          return FieldConstants.FieldSection.BLUE_HUB;
        }
        else if (y > 5.4  && (x > 4 && x < 8.4)){
          return FieldConstants.FieldSection.BLUE_LEFT;
        }
        else if (y < 2.6 && (x > 4 && x < 8.4)){
          return FieldConstants.FieldSection.BLUE_RIGHT;
        }
        else if ((y <5.4 && y > 2.6)  && (x > 4 && x < 8.4)){
          return FieldConstants.FieldSection.BLUE_MIDDLE;
        }

        else if (x > 12.5){
          return FieldConstants.FieldSection.RED_HUB;
        }
        else if (y > 5.4  && (x > 8.4 && x < 12.5)){
          return FieldConstants.FieldSection.RED_RIGHT;
        }
        else if (y < 2.6 && (x > 8.4 && x < 12.5)){
          return FieldConstants.FieldSection.RED_LEFT;
        }
        else if ((y < 5.4 && y > 2.6)  && (x > 8.4 && x < 12.5)){
          return FieldConstants.FieldSection.RED_MIDDLE;
        }
        else{
          return FieldConstants.FieldSection.UNKNOWN;
        }
            
    }



  private Pose3d ghostTargetPose(Pose3d targetPose, Pose3d botPose3d, SwerveSubsystem swerveSubsystem){
     // Inputs
      double botX = botPose3d.getX();
      double botY = botPose3d.getY();

      double botVx = swerveSubsystem.getVelocity().vxMetersPerSecond;
      double botVy = swerveSubsystem.getVelocity().vyMetersPerSecond;

      double targetX = targetPose.getX();
      double targetY = targetPose.getY();

      double v = CalcVelocity(targetPose, botPose3d);
      double bulletSpeed = v * Math.cos(Constants.TurretConstants.launchAngle);

      // Relative position from bot to target
      double relX = targetX - botX;
      double relY = targetY - botY;

      // Precompute scalars
      double botV2 = botVx * botVx + botVy * botVy;   // |V|^2
      double relDotV = relX * botVx + relY * botVy;   // R·V
      double rel2 = relX * relX + relY * relY;        // |R|^2

      // Quadratic coefficients
      double a = bulletSpeed * bulletSpeed - botV2;
      double b = 2.0 * relDotV;
      double c = -rel2;

      // Discriminant
      double disc = b * b - 4.0 * a * c;
      if (disc < 0.0) {
          // No solution — bullet too slow to intercept
          // fallback behavior
      }

      double sqrtDisc = Math.sqrt(disc);
      double t1 = (-b + sqrtDisc) / (2.0 * a);
      double t2 = (-b - sqrtDisc) / (2.0 * a);

      // Choose smallest positive intercept time
      double t = -1.0;
      if (t1 > 0.0 && t2 > 0.0) t = Math.min(t1, t2);
      else if (t1 > 0.0) t = t1;
      else if (t2 > 0.0) t = t2;

      if (t <= 0.0) {
          // No valid positive intercept time
          // fallback behavior
      }

      // Virtual target (lead point)
      double virtualX = targetX - botVx * t;
      double virtualY = targetY - botVy * t;

      // Aim turret at (virtualX, virtualY)

      return new Pose3d(virtualX, virtualY, targetPose.getZ(), targetPose.getRotation());
  }

/*
   * calculates the required velocity needed to hit the set point
   * 
   * @param targetLocation
   */
  public double CalcVelocity(Pose3d TargetLocation, Pose3d RobotLocation){
    double g = 9.8;
    double height = TargetLocation.getZ() - RobotLocation.getZ();

    double xDistance = Math.abs(TargetLocation.getX() - RobotLocation.getX());
    double yDistance = Math.abs(TargetLocation.getY()- RobotLocation.getY());
    double distance = Math.sqrt(Math.pow(xDistance, 2) + Math.pow(yDistance, 2));

    double numerator = g*Math.pow(distance, 2);
    double denominator = 2*Math.pow(Math.cos(Constants.TurretConstants.launchAngle), 2)*(distance * Math.tan(Constants.TurretConstants.launchAngle) - height);

    double velocity = Math.sqrt(numerator/denominator);

    return velocity;
  }

  /** Creates a new ExampleSubsystem. */
  public TurretSubsystem(double botRelativeXPos, double botRelativeYPos, TurretSide turretSide) {
    this.botRelativeXPos = botRelativeXPos;
    this.botRelativeYPos = botRelativeYPos;
    this.turretSide = turretSide;

    
  }

 
  @Override
  public void periodic() {
    turrePivot.updateTelemetry();
    
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    turrePivot.simIterate();
    // DogLog.log("Turret Angle", angle);
  }
}
