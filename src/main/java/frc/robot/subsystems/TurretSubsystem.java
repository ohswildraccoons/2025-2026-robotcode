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

import java.lang.annotation.Target;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.revrobotics.spark.SparkLowLevel.MotorType;

 import dev.doglog.DogLog;

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
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.gearing.Sprocket;
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

  int turretId;
  double angle = 0;
  double botRelativeXPos;
  double botRelativeYPos;
  Pose3d turretFieldRelativePose;
  Boolean manual = false;
  BooleanSupplier isManualSetpointTargeting = () ->manual;
  public enum TurretSide {
    LEFT,
    RIGHT
  }
  TurretSide turretSide;
  Pose3d currentlyTargetedPose = Constants.FieldConstants.middleField;
  Supplier<Pose3d> target = () -> currentlyTargetedPose;
  Pose3d currentGhostTarget = Constants.FieldConstants.middleField;
  // private final ShooterSubsytem m_shooterSubsystem = new ShooterSubsytem();

    
  private SmartMotorControllerConfig smcConfig = new SmartMotorControllerConfig(this)
  .withControlMode(ControlMode.CLOSED_LOOP)
  .withClosedLoopController(0.5, 0.00, 0.0)
  //.withClosedLoopController(4, 0, 0, DegreesPerSecond.of(180), DegreesPerSecondPerSecond.of(90)) Profiled PID breaks the thing?! - hs 20JAN
//  .withSimClosedLoopController(4.0, 0, 0, DegreesPerSecond.of(180), DegreesPerSecondPerSecond.of(90))
  // Configure Motor and Mechanism properties
  .withGearing(new MechanismGearing(GearBox.fromReductionStages(5,1),Sprocket.fromStages("12:72")))//
  .withIdleMode(MotorMode.BRAKE)
  .withMotorInverted(false)
  // Setup Telemetry
  .withTelemetry("TurretMotor", TelemetryVerbosity.HIGH)
  // Power Optimization
  .withStatorCurrentLimit(Amps.of(30))
  .withClosedLoopRampRate(Seconds.of(0.25))
  .withOpenLoopRampRate(Seconds.of(0.25));

  
  // Vendor motor controller object
  private SparkMax spark;

  // Create our SmartMotorController from our Spark and config with the NEO.
  private SmartMotorController sparkSmartMotorController;

  PivotConfig                m_config;
     

  // Arm Mechanism
  private Pivot turrePivot;

   /** Creates a new ExampleSubsystem. */
  public TurretSubsystem(double botRelativeXPos, double botRelativeYPos, TurretSide turretSide, int turretId) {
    this.botRelativeXPos = botRelativeXPos;
    this.botRelativeYPos = botRelativeYPos;
    this.turretSide = turretSide;
    this.turretId = turretId;
    String name = (turretSide == TurretSide.LEFT) ? "Left turret" : "right Turret";

    spark = new SparkMax(turretId, MotorType.kBrushless);
    sparkSmartMotorController= new SparkWrapper(spark, DCMotor.getNEO(1), smcConfig);
    m_config= new PivotConfig(sparkSmartMotorController)
      .withStartingPosition(Degrees.of(0)) // Starting position of the Pivot
      // .withWrapping(Degrees.of(-180), Degrees.of(180)) // Wrapping disabled bc the pivot cant spin infinitely
      .withHardLimit(Degrees.of(-90), Degrees.of(90)) // Hard limit bc wiring prevents infinite spinning
      .withTelemetry(name, TelemetryVerbosity.HIGH) // Telemetry
      .withMOI(Feet.of(0.25), Pounds.of(4)); // MOI Calculation
    turrePivot = new Pivot(m_config);
  }


  /**
   * Set the angle of the arm.
   * @param angle Angle to go to.
   */
  public Command setAngle(Supplier<Angle> angle) {
    return turrePivot.setAngle(angle);
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
         currentGhostTarget = ghostTargetPose(targetPose.get(), turretFieldRelativePose, swerveSubsystem);
        double deltaX = currentGhostTarget.getX() - turretFieldRelativePose.getX();
        double deltaY = currentGhostTarget.getY() - turretFieldRelativePose.getY();
        double distance = Math.sqrt(Math.pow(deltaX, 2) + Math.pow(deltaY, 2));

        double angle = Math.atan2(deltaY, deltaX);
        double angleDeg = adjustAngleToFieldRelative(robotPose, () -> Math.toDegrees(angle));
        SmartDashboard.putNumber("Turret Target Angle", angleDeg);
        
        SmartDashboard.putNumber("v-target x", currentGhostTarget.getX());
        SmartDashboard.putNumber("v-target y", currentGhostTarget.getY());


        SmartDashboard.putNumber("angle of Turret", angleDeg);
        return Degrees.of(angleDeg);
      });
    }

    public double adjustAngleToFieldRelative(Supplier<Pose3d> robotPose, Supplier<Double> angle) {
        double robotAngleDeg = Math.toDegrees(robotPose.get().getRotation().getZ());
        return angle.get() - robotAngleDeg;
    }

    public Command autoFindTargetPose(Supplier<Pose3d> robotPose, SwerveSubsystem swerveSubsystem, Boolean isBlueAlliance) {

        Supplier<Pose3d> targetPose = () -> {
          if (isBlueAlliance) {
            switch (getFieldSection(robotPose.get())) {

                case BLUE_HUB:
                    return FieldConstants.blueHub;

                case RED_HUB:
                case RED_LEFT:
                case RED_RIGHT:
                case RED_MIDDLE:
                    return (turretSide == TurretSide.LEFT)
                            ? FieldConstants.blueLeftDeposit
                            : FieldConstants.blueRightDeposit;

                case BLUE_LEFT:
                    return FieldConstants.blueLeftDeposit;

                case BLUE_RIGHT:
                    return FieldConstants.blueRightDeposit;

                case BLUE_MIDDLE:
                    return (turretSide == TurretSide.LEFT)
                            ? FieldConstants.blueLeftDeposit
                            : FieldConstants.blueRightDeposit;

                default:
                    return FieldConstants.middleField;
            }

        } else { // RED ALLIANCE

            switch (getFieldSection(robotPose.get())) {

                case RED_HUB:
                    return FieldConstants.redHub;

                case BLUE_HUB:
                case BLUE_LEFT:
                case BLUE_RIGHT:
                case BLUE_MIDDLE:
                    return (turretSide == TurretSide.LEFT)
                            ? FieldConstants.redLeftDeposit
                            : FieldConstants.redRightDeposit;

                case RED_LEFT:
                    return FieldConstants.redLeftDeposit;

                case RED_RIGHT:
                    return FieldConstants.redRightDeposit;

                case RED_MIDDLE:
                    return (turretSide == TurretSide.LEFT)
                            ? FieldConstants.redLeftDeposit
                            : FieldConstants.redRightDeposit;

                default:
                    return FieldConstants.middleField;
            }
        }

            
        };
          return targetPose(robotPose, targetPose, swerveSubsystem);
    }

    public Command targettingCommand(Supplier<Pose3d> robotPose, SwerveSubsystem swerveSubsystem, Alliance alliance) {
// // TODO
        boolean isBlueAlliance = (alliance == Alliance.Red) ? false : true;
        return new ConditionalCommand(
            targetPose(robotPose, target, swerveSubsystem),
            autoFindTargetPose(robotPose, swerveSubsystem, isBlueAlliance),
            isManualSetpointTargeting
        );

    }

    public Supplier<Pose3d> getGhostSupplier(){
      return ()-> currentGhostTarget;
    }
    
    public Supplier<Pose3d> getTurretFieldPosSupplier(Supplier<Pose3d> botPose3d){
      return (turretFieldRelativePose != null) ?  () ->turretFieldRelativePose: botPose3d;
    }


    public Command setAutoTargettingOn() {
        return runOnce(() -> {
            manual = false;
        });
    }

    public Command setManualTarget(Pose3d target) {
        return runOnce(() -> {
            currentlyTargetedPose = target;
            manual = true;
            SmartDashboard.putString("Target x", Double.toString(target.getX()));
            SmartDashboard.putString("Target y", Double.toString(target.getY()));
        });
    }

    public Command setSplitTarget(Alliance alliance){
      if(alliance == Alliance.Blue){
        if (turretSide == TurretSide.LEFT){
          return setManualTarget(Constants.FieldConstants.blueLeftDeposit);
        }else {
          return setManualTarget(Constants.FieldConstants.blueRightDeposit);
        }
      }else if(alliance ==Alliance.Red){
        if (turretSide == TurretSide.LEFT){
          return setManualTarget(Constants.FieldConstants.redLeftDeposit);
        }else {
          return setManualTarget(Constants.FieldConstants.redRightDeposit);
        }
      } else{
        if (turretSide == TurretSide.LEFT){
          return setManualTarget(currentlyTargetedPose);
        }else {
          return setManualTarget(currentlyTargetedPose);
        }
      }
    }

    public Supplier<Pose3d> getTarget(){
        return target;
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



private Pose3d ghostTargetPose(Pose3d targetPose, Pose3d botPose3d, SwerveSubsystem swerveSubsystem) {
    double botX = botPose3d.getX();
    double botY = botPose3d.getY();

    // Robot velocity (field-relative)
    double botVx = swerveSubsystem.getVelocity().vxMetersPerSecond;
    double botVy = swerveSubsystem.getVelocity().vyMetersPerSecond;

    // Target position (stationary)
    double targetX = targetPose.getX();
    double targetY = targetPose.getY();

    // Target diff from robot
    double targetDiffX = targetX - botX;
    double targetDiffY = targetY - botY; 
    double alpha = Math.atan2(targetDiffY, targetDiffX);

    double velocity = CalcVelocity(targetPose, botPose3d);
    double shooterAngle = Constants.TurretConstants.launchAngle;

    double parallelToGroundVelocity = velocity * Math.cos(shooterAngle);
    double parallelToGroundVelocityX = parallelToGroundVelocity * Math.cos(alpha);
    double parallelToGroundVelocityY = parallelToGroundVelocity * Math.sin(alpha);

    double requiredBulletVelocityX = parallelToGroundVelocityX - botVx;
    double requiredBulletVelocityY = parallelToGroundVelocityY - botVy;
    double requiredTurretAngle = Math.atan2(requiredBulletVelocityY,requiredBulletVelocityX);

    double t = parallelToGroundVelocityX / targetDiffX; 

    Pose3d ghostPose = new Pose3d((requiredBulletVelocityX * t) + botX, (requiredBulletVelocityY * t) + botY, targetPose.getZ(), targetPose.getRotation());

  SmartDashboard.putNumber("velocity", velocity);
  SmartDashboard.putNumber("shooterAngle", shooterAngle);

  SmartDashboard.putNumber("parallelToGroundVelocity", parallelToGroundVelocity);
  SmartDashboard.putNumber("parallelToGroundVelocityX", parallelToGroundVelocityX);
  SmartDashboard.putNumber("parallelToGroundVelocityY", parallelToGroundVelocityY);
  SmartDashboard.putNumber("alpha", 180*alpha/3.145);
  SmartDashboard.putNumber("requiredTurretAngle", 180*requiredTurretAngle/3.145);
  
  SmartDashboard.putNumber("Ghost Pose X", ghostPose.getX());
  SmartDashboard.putNumber("Ghost Pose Y", ghostPose.getY());
  SmartDashboard.putNumber("Target Pose X", targetPose.getX());
  SmartDashboard.putNumber("Target Pose Y", targetPose.getY());

    return ghostPose;

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

 


 
  @Override
  public void periodic() {
    turrePivot.updateTelemetry();
    SmartDashboard.putBoolean("Turret Manual Targeting", isManualSetpointTargeting.getAsBoolean());
    SmartDashboard.putString("Turret Angle", turrePivot.getAngle().toShortString());
    SmartDashboard.putNumber("Turret angle var",  angle);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    turrePivot.simIterate();
    // if(angle != null){ // Attempting this? -HS
    // DogLog.log("Turret Angle", angle);

    }

public void setCurrentLimit(double limit) {
    turrePivot.getMotorController().setSupplyCurrentLimit(Amps.of(limit));
}
  }
