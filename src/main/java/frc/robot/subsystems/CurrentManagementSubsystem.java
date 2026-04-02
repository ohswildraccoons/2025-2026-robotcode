// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

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
 int nonBalancedTimer;

SwerveSubsystem managedDrivetrain;
IntakeSubsystem managedIntake; 
serializerSubsystem managedSerializer; 
TurretSubsystem managedLeftTurret;
TurretSubsystem managedRightTurret; 
ShooterSubsytem managedLeftShooter;
ShooterSubsytem managedrightShooter;

 

  /** Creates a new ExampleSubsystem. */
  public CurrentManagementSubsystem(SwerveSubsystem drivetrain, IntakeSubsystem intake, serializerSubsystem serializer, TurretSubsystem LeftTurret, TurretSubsystem RightTurret, ShooterSubsytem LeftShooter, ShooterSubsytem rightShooter)// Adding subsystems in constructor because they are private and also getters dont work {
 {
    CurrentProfile activeCurrentProfile = null;
    CurrentProfile nexCurrentProfile = activeCurrentProfile;
    CurrentProfileDisplay = new Alert(activeCurrentProfile.toString(), AlertType.kInfo);

    managedDrivetrain=drivetrain;
    managedIntake=intake;
    managedSerializer = serializer;
    managedLeftTurret = LeftTurret;
    managedRightTurret = RightTurret;
    managedLeftShooter =LeftShooter;
    managedrightShooter = rightShooter;
    nonBalancedTimer = 250; // 5 econds, 250 robotperiodic iteration
   
   

  }

 


  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command ApplyCurrentLimitProfile(CurrentProfile input) {
    // Subsystem::RunOnce implicitly requires `this` subsystem.

/* 	

Min breaker blo curve from optifue (i kno, eaton no make graph)
Time Amps
0.2	972
0.5	708
1	492
5	300
10	240
100	132
135	126
*/

    return runOnce(
        () -> {
         
          switch (input)
          {
             
            case BALANCED:
            //Balanced is "normal" using default limits. Right now, aiming for 
            //TODO: Tune these values - current maxx iaa 400... may have to get eperimental on this: get drivetrain acrual value from a match log, figure out idle and under load at FULLSEND for each mechanism. 
              managedDrivetrain.setCurrentLimit(40, 20);
              managedIntake.setCurrentLimit(20.0);
              managedSerializer.setCurrentLimit(20.0);
              managedLeftTurret.setCurrentLimit(10.0);
              managedRightTurret.setCurrentLimit(10.0);
              managedLeftShooter.setCurrentLimit(20.0);
              managedrightShooter.setCurrentLimit(20.0);
              CurrentProfileDisplay.setText(activeCurrentProfile.toString());
              nonBalancedTimer = 250;

              break;

            case SHOOTER: 
              managedDrivetrain.setCurrentLimit(15, 10);  //100A
              managedIntake.setCurrentLimit(20.0);
              managedSerializer.setCurrentLimit(20.0);
              managedLeftTurret.setCurrentLimit(8.0);
              managedRightTurret.setCurrentLimit(8.0);
              managedLeftShooter.setCurrentLimit(35.0);
              managedrightShooter.setCurrentLimit(35.0);
              CurrentProfileDisplay.setText(activeCurrentProfile.toString());
              nonBalancedTimer = 250;

          
              break;

            case DRIVE:
              managedDrivetrain.setCurrentLimit(60, 25);  //100A
              managedIntake.setCurrentLimit(20.0);
              managedSerializer.setCurrentLimit(10.0);
              managedLeftTurret.setCurrentLimit(5.0);
              managedRightTurret.setCurrentLimit(5.0);
              managedLeftShooter.setCurrentLimit(5.0);
              managedrightShooter.setCurrentLimit(10.0);
              CurrentProfileDisplay.setText(activeCurrentProfile.toString());
              nonBalancedTimer = 250;
              
              

            //put limit apps here
              break;

            case INTAKE:

              managedDrivetrain.setCurrentLimit(30, 15);
              managedIntake.setCurrentLimit(30.0);
              managedSerializer.setCurrentLimit(30.0);
              managedLeftTurret.setCurrentLimit(5.0);
              managedRightTurret.setCurrentLimit(5.0);
              managedLeftShooter.setCurrentLimit(10.0);
              managedrightShooter.setCurrentLimit(10.0);
              CurrentProfileDisplay.setText(activeCurrentProfile.toString());
              nonBalancedTimer = 250;

            //put limit apps here
              break;

              case FULLSEND:

              CurrentProfileDisplay = new Alert(activeCurrentProfile.toString(), AlertType.kError);

              
              managedDrivetrain.setCurrentLimit(30, 15);
              managedIntake.setCurrentLimit(30.0);
              managedSerializer.setCurrentLimit(30000.0);
              managedLeftTurret.setCurrentLimit(5000.0);
              managedRightTurret.setCurrentLimit(5000.0);
              managedLeftShooter.setCurrentLimit(10000.0);
              managedrightShooter.setCurrentLimit(10000.0);
              CurrentProfileDisplay.setText(activeCurrentProfile.toString());
              nonBalancedTimer = 250;
              
              nonBalancedTimer=-1;

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
     if (nonBalancedTimer==0)
     {
      nextCurrentProfile=CurrentProfile.BALANCED;
     }

      if(nextCurrentProfile!=activeCurrentProfile)
      {
        ApplyCurrentLimitProfile(nextCurrentProfile);
      }

    nonBalancedTimer--;

    
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}