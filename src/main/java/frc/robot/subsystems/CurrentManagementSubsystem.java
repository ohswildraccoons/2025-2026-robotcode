// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CurrentManagementSubsystem extends SubsystemBase {

  //Define subsystems
  //Define profiles

  enum CurrentProfile {
 BALANCED, SHOOTER, DRIVE, INTAKE, FULLSEND
 }
 CurrentProfile nextCurrentProfile;
CurrentProfile activeCurrentProfile;

 Alert CurrentProfileDisplay;


 int balancedDriveLimit;
 int balancedShooterLimit;
 int balancedIntakeLimit;

 

  /** Creates a new ExampleSubsystem. */
  public CurrentManagementSubsystem() {

    CurrentProfile activeCurrentProfile = CurrentProfile.BALANCED;
    CurrentProfile nexCurrentProfile = activeCurrentProfile;
    CurrentProfileDisplay = new Alert(activeCurrentProfile.toString(), AlertType.kInfo);
  }

  public CurrentManagementSubsystem(CurrentProfile input) {

    CurrentProfile activeCurrentProfile = input;
    CurrentProfile nexCurrentProfile = activeCurrentProfile;
    CurrentProfileDisplay = new Alert(activeCurrentProfile.toString(), AlertType.kInfo);


  }


  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command ApplyCurrentLimitProfile(CurrentProfile input) {
    // Subsystem::RunOnce implicitly requires `this` subsystem.

    return runOnce(
        () -> {
         
          switch (input)
          {
             
            case BALANCED:

            //put limit apps here
              break;

              case SHOOTER:

            //put limit apps here
              break;

              case DRIVE:
              

            //put limit apps here
              break;

              case INTAKE:

            //put limit apps here
              break;

              case FULLSEND:

              CurrentProfileDisplay = new Alert(activeCurrentProfile.toString(), AlertType.kWarning);

            //put limit apps here
              break;
          
            default:
              break;
          }



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
    //TODO: Comparator to only set when a change has been called, then call that change

      if(nextCurrentProfile!=activeCurrentProfile)
      {
        ApplyCurrentLimitProfile(nextCurrentProfile);
      }

    
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}