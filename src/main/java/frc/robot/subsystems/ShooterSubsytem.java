  package frc.robot.subsystems;


  import static edu.wpi.first.units.Units.Amps;
  import static edu.wpi.first.units.Units.DegreesPerSecond;
  import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
  import static edu.wpi.first.units.Units.Feet;
  import static edu.wpi.first.units.Units.Inches;
  import static edu.wpi.first.units.Units.Pounds;
  import static edu.wpi.first.units.Units.Second;
  import static edu.wpi.first.units.Units.Seconds;
  import static edu.wpi.first.units.Units.Volts;

  import java.util.function.Supplier;

  import static edu.wpi.first.units.Units.RPM;

  import frc.robot.Constants;
  import frc.robot.Constants.MotorConstants;

  import com.revrobotics.spark.SparkLowLevel.MotorType;
  import com.ctre.phoenix6.controls.Follower;
  import com.ctre.phoenix6.hardware.TalonFX;
  import com.ctre.phoenix6.hardware.TalonFXS;
  import com.ctre.phoenix6.signals.MotorAlignmentValue;
  import com.revrobotics.spark.SparkMax;

  import edu.wpi.first.math.controller.SimpleMotorFeedforward;
  import edu.wpi.first.math.geometry.Pose3d;
  import edu.wpi.first.math.system.plant.DCMotor;
  import edu.wpi.first.units.measure.AngularVelocity;
  import edu.wpi.first.units.measure.Distance;
  import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
  import edu.wpi.first.wpilibj2.command.SubsystemBase;
  import yams.gearing.GearBox;
  import yams.gearing.MechanismGearing;
  import yams.mechanisms.SmartMechanism;
  import yams.mechanisms.config.FlyWheelConfig;
  import yams.mechanisms.velocity.FlyWheel;
  import yams.motorcontrollers.SmartMotorController;
  import yams.motorcontrollers.local.SparkWrapper;
  import yams.motorcontrollers.remote.TalonFXSWrapper;
  import yams.motorcontrollers.remote.TalonFXWrapper;
  import yams.motorcontrollers.SmartMotorControllerConfig;
  import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
  import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
  import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;

  public class ShooterSubsytem extends SubsystemBase  {

    int ShooterMotorIDLeft;
    int ShooterMotorIDRight;
    
          
      private SmartMotorControllerConfig smcConfig = new SmartMotorControllerConfig(this)
      .withControlMode(ControlMode.CLOSED_LOOP)

      // Feedback Constants (PID Constants)
      .withClosedLoopController(1.5, 0, 0, DegreesPerSecond.of(900000), DegreesPerSecondPerSecond.of(9999999))
      .withSimClosedLoopController(0.8, 0, 0, DegreesPerSecond.of(900000), DegreesPerSecondPerSecond.of(45))

      // Feedforward Constants
      .withFeedforward(new SimpleMotorFeedforward(0, 0.11, 0))
      .withSimFeedforward(new SimpleMotorFeedforward(0, 0, 0))

      // Telemetry name and verbosity level
      .withTelemetry("ShooterMotor", TelemetryVerbosity.HIGH)

      // Gearing from the motor rotor to final shaft.
      // In this example GearBox.fromReductionStages(3,4) is the same as GearBox.fromStages("3:1","4:1") which corresponds to the gearbox attached to your motor.
      // You could also use .withGearing(12) which does the same thing.
      .withGearing(new MechanismGearing(GearBox.fromStages("1:1")))

      // Motor properties to prevent over currenting.
      .withMotorInverted(false)
      .withIdleMode(MotorMode.COAST)
      .withSupplyCurrentLimit(Amps.of(40))
      .withStatorCurrentLimit(Amps.of(60)); 


      
      // Vendor motor controller object
      private TalonFX talonLeft;
      private TalonFX talonRight;
      private TalonFXWrapper shooterMotor;
    
      private final FlyWheelConfig shooterConfig;

      // Shooter Mechanism
      private FlyWheel shooter;


    /**
     * Gets the current velocity of the shooter.
     *
     * @return Shooter velocity.
     */
    public AngularVelocity getVelocity() {return shooter.getSpeed();}

      /**
     * Set the shooter velocity.
     *
     * @param speed Speed to set.
     * @return {@link edu.wpi.first.wpilibj2.command.RunCommand}
     */
    public Command setSpeed(AngularVelocity speed) {return runOnce( () -> {shooter.setMechanismVelocitySetpoint(speed);});}
    
    public Command setVelocityOfFire(double velocity) {
      
    double distanceGround = findDistanceBasedOnVelocity(velocity);

    double prevDistance = 0;
    double previousRPM = 0;
    int id = 0;
    

    for (double distance: Constants.ShooterConstants.shooterDistances) {
      if (distance == distanceGround){
        return shooter.setSpeed(RPM.of(Constants.ShooterConstants.shooterRPMs[id]));
      }else if (distance >= distanceGround){
        double slope = (distance - prevDistance)/(Constants.ShooterConstants.shooterRPMs[id] - previousRPM);
        double interpolatedRPM = previousRPM + slope * (distanceGround - prevDistance);
        return shooter.setSpeed(RPM.of(interpolatedRPM));
      }else{
        prevDistance = distance;
        previousRPM = Constants.ShooterConstants.shooterRPMs[id];
        id++;
      }
    }
    return shooter.setSpeed(RPM.of(Constants.ShooterConstants.shooterRPMs[Constants.ShooterConstants.shooterRPMs.length - 1]));
    }

    public Command autoSetVelocityOfFire(Supplier<Pose3d> TargetLocation, Supplier<Pose3d> shooterLocation) {
      double velocity = CalcVelocity(TargetLocation.get(), shooterLocation.get());
      return setVelocityOfFire(velocity);
    }

    /*
    * calculates the required velocity needed to hit the set point
    * 
    * @param targetLocation
    */
    public double CalcVelocity(Pose3d TargetLocation, Pose3d shooterLocation){
      double g = 9.8;
      double height = TargetLocation.getZ() - shooterLocation.getZ();

      double xDistance = Math.abs(TargetLocation.getX() - shooterLocation.getX());
      double yDistance = Math.abs(TargetLocation.getY()- shooterLocation.getY());
      double distance = Math.sqrt(Math.pow(xDistance, 2) + Math.pow(yDistance, 2));

      double numerator = g*Math.pow(distance, 2);
      double denominator = 2*Math.pow(Math.cos(Constants.TurretConstants.launchAngle), 2)*(distance * Math.tan(Constants.TurretConstants.launchAngle) - height);

      double velocity = Math.sqrt(numerator/denominator);

      return velocity;
    }

    /*
    * Calculates given a distance in the air, the distance it will be on the ground
    * 
    * @param target Location
    * @param RobotLocation
    */
    public double findDistanceBasedOnVelocity(double velocity){
      double distance = (Math.pow(velocity, 2)*Math.sin(2*Constants.TurretConstants.launchAngle))/9.8;
      return distance;
    }

    /**
     * Set the dutycycle of the shooter.
     *
     * @param dutyCycle DutyCycle to set.
     * @return {@link edu.wpi.first.wpilibj2.command.RunCommand}
     */
    public Command set(double dutyCycle) {return shooter.set(dutyCycle);}

    /** Creates a new ExampleSubsystem. */
    public ShooterSubsytem(int ShooterMotorIDLeft, int ShooterMotorIDRight) {
      this.ShooterMotorIDLeft = ShooterMotorIDLeft;
      this.ShooterMotorIDRight = ShooterMotorIDRight;

      talonLeft = new TalonFX(ShooterMotorIDLeft);
      talonRight = new TalonFX(ShooterMotorIDRight);

      // YAMS only wraps ONE motor
      shooterMotor = new TalonFXWrapper(
          talonLeft,
          DCMotor.getKrakenX44(1),
          smcConfig
      );

      // Right motor follows left motor, inverted
      talonRight.setControl(new Follower(ShooterMotorIDLeft, MotorAlignmentValue.Opposed));

      shooterConfig = new FlyWheelConfig(shooterMotor)
      // Diameter of the flywheel.
      .withDiameter(Inches.of(Constants.ShooterConstants.shooterWheelRadiusInches))
      // Mass of the flywheel.       
      .withMass(Pounds.of(1))
      // Maximum speed of the shooter.
      
      .withUpperSoftLimit(RPM.of(1000)) //TODO: this might need to be higher
      // Telemetry name and verbosity for the arm.  
      .withTelemetry("ShooterMech", TelemetryVerbosity.HIGH);

      shooter = new FlyWheel(shooterConfig);

    }

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
      shooter.updateTelemetry();
      
    }

    @Override
    public void simulationPeriodic() {
      // This method will be called once per scheduler run during simulation
      shooter.simIterate();
    }
  }
      