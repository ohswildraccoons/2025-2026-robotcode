package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CurrentManagementSubsystem extends SubsystemBase {

    public enum CurrentProfile {
        BALANCED,
        SHOOTER,
        DRIVE,
        INTAKE,
        FULLSEND
    }

    private CurrentProfile activeProfile;
    private CurrentProfile nextProfile;

    private Alert profileAlert;

    private int nonBalancedTimer = 0;

    // Managed subsystems (matching YOUR names)
    private final SwerveSubsystem drivetrain;
    private final IntakeSubsystem intake;
    private final serializerSubsystem serializer;
    private final TurretSubsystem leftTurret;
    private final TurretSubsystem rightTurret;
    private final ShooterSubsytem leftShooter;
    private final ShooterSubsytem rightShooter;

    public CurrentManagementSubsystem(
            SwerveSubsystem drivetrain,
            IntakeSubsystem intake,
            serializerSubsystem serializer,
            TurretSubsystem leftTurret,
            TurretSubsystem rightTurret,
            ShooterSubsytem leftShooter,
            ShooterSubsytem rightShooter) {

        this.drivetrain = drivetrain;
        this.intake = intake;
        this.serializer = serializer;
        this.leftTurret = leftTurret;
        this.rightTurret = rightTurret;
        this.leftShooter = leftShooter;
        this.rightShooter = rightShooter;

        activeProfile = CurrentProfile.DRIVE;
        nextProfile = activeProfile;

        profileAlert = new Alert("Current Profile: " + activeProfile, AlertType.kInfo);
    }

    public Command applyDriveProfile() {
        return applyProfile(CurrentProfile.DRIVE);
    }

    public Command applyProfile(CurrentProfile profile) {
        return runOnce(() -> applyProfileInternal(profile));
    }

    private void applyProfileInternal(CurrentProfile profile) {

        activeProfile = profile;
        nextProfile = profile;

        switch (profile) {

            case BALANCED:
                drivetrain.setCurrentLimit(40, 20);
                intake.setCurrentLimit(20);
                // serializer.setCurrentLimit(20);
                leftTurret.setCurrentLimit(10);
                rightTurret.setCurrentLimit(10);
                leftShooter.setCurrentLimit(20);
                rightShooter.setCurrentLimit(20);
                nonBalancedTimer = 250;
                break;

            case SHOOTER:
                drivetrain.setCurrentLimit(15, 10);
                intake.setCurrentLimit(20);
                serializer.setCurrentLimit(20);
                leftTurret.setCurrentLimit(8);
                rightTurret.setCurrentLimit(8);
                leftShooter.setCurrentLimit(35);
                rightShooter.setCurrentLimit(35);
                nonBalancedTimer = 250;
                break;

            case DRIVE:
                drivetrain.setCurrentLimit(60, 25);
                intake.setCurrentLimit(20);
                serializer.setCurrentLimit(10);
                leftTurret.setCurrentLimit(5);
                rightTurret.setCurrentLimit(5);
                leftShooter.setCurrentLimit(5);
                rightShooter.setCurrentLimit(10);
                nonBalancedTimer = 250;
                break;

            case INTAKE:
                drivetrain.setCurrentLimit(30, 15);
                intake.setCurrentLimit(30);
                serializer.setCurrentLimit(30);
                leftTurret.setCurrentLimit(5);
                rightTurret.setCurrentLimit(5);
                leftShooter.setCurrentLimit(10);
                rightShooter.setCurrentLimit(10);
                nonBalancedTimer = 250;
                break;

            case FULLSEND:
                profileAlert = new Alert("FULLSEND MODE ACTIVE", AlertType.kError);

                drivetrain.setCurrentLimit(90, 35);
                intake.setCurrentLimit(30);
                serializer.setCurrentLimit(30000);
                leftTurret.setCurrentLimit(5000);
                rightTurret.setCurrentLimit(5000);
                leftShooter.setCurrentLimit(10000);
                rightShooter.setCurrentLimit(10000);

                nonBalancedTimer = -1;
                break;
        }

        profileAlert.setText("Current Profile: " + activeProfile);
    }

    @Override
    public void periodic() {

        if (nonBalancedTimer == 0) {
            nextProfile = CurrentProfile.BALANCED;
        }

        if (nextProfile != activeProfile) {
            applyProfileInternal(nextProfile);
        }

        if (nonBalancedTimer > 0) {
            nonBalancedTimer--;
        }
    }
}