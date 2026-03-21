
package frc.robot.subsystems;

import java.util.Optional;

import frc.robot.Constants.CameraConstants;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;


import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;


public class CameraSubsystem extends SubsystemBase {
    private static CameraSubsystem m_instance;
  
    PhotonPoseEstimator photonPoseEstimatorFR;
    PhotonPoseEstimator photonPoseEstimatorFL;
    PhotonPoseEstimator photonPoseEstimatorBR;
    PhotonPoseEstimator photonPoseEstimatorBL;
    PhotonPoseEstimator photonPoseEstimatorA;
  
    Pose2d previPose2d = new Pose2d();
  
    AutoMode m_autoMode = AutoMode.kOff;

    int latestFiducial = 0;
    long latestDetected = 0;

    public static enum AutoMode {
      kOff,
      kReef,
      kCoralStation
    }

    public static enum Camera {
      FR,
      FL,
      BR,
      BL,
    }
  
    public static CameraSubsystem getInstance() {
      if (m_instance == null) {
        m_instance = new CameraSubsystem();
      }
  
      return m_instance;
    }
    
    PhotonCamera cameraFR;
    PhotonCamera cameraFL;
    PhotonCamera cameraBR;
    PhotonCamera cameraBL;

    /** Creates a new CameraSubsystem. */
    private CameraSubsystem() {
      super();
      // The field from AprilTagFields will be different depending on the game.
  
      // Gets the 2025 Welded AprilTag layout (change to AndyMark if not in the U.S.)
      AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
     
      //creating two cameras and assigning them a position relative to the center of the bot
      cameraFR = new PhotonCamera("Camera FR");
      Transform3d robotToCamFR = new Transform3d(
        new Translation3d(CameraConstants.frontRightX, CameraConstants.frontRightY , CameraConstants.frontRightZ),
        new Rotation3d(CameraConstants.frontRightRoll, CameraConstants.frontRightPitch, CameraConstants.frontRightYaw)
      ); //Cam mounted facing forward, upper left of the robot
      cameraFL = new PhotonCamera("Camera FL");
      Transform3d robotToCamFL = new Transform3d(
        new Translation3d(CameraConstants.frontLeftX, CameraConstants.frontLeftY, CameraConstants.frontLeftZ),
        new Rotation3d(CameraConstants.frontLeftRoll, CameraConstants.frontLeftPitch, CameraConstants.frontLeftYaw)
      ); //Cam mounted facing forward, upper right of the robot
      cameraBR = new PhotonCamera("Camera BR");
      Transform3d robotToCamBR = new Transform3d(
        new Translation3d(CameraConstants.backRightX, CameraConstants.backRightY, CameraConstants.backRightZ),
        new Rotation3d(CameraConstants.backRightRoll, CameraConstants.backRightPitch, CameraConstants.backRightYaw)
      ); //Cam mounted facing forward, lower left of the robot
      cameraBL = new PhotonCamera("Camera BL");
      Transform3d robotToCamBL = new Transform3d(
        new Translation3d(CameraConstants.backLeftX, CameraConstants.backLeftY, CameraConstants.backLeftZ),
        new Rotation3d(CameraConstants.backLeftRoll, CameraConstants.backLeftPitch, CameraConstants.backLeftYaw)
      ); //Cam mounted facing forward, in the bottom center of the robot
    
      //feeding in info for camera postion to photon pose estimator
      photonPoseEstimatorFR = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToCamFR);
      photonPoseEstimatorFL = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToCamFL);
      photonPoseEstimatorBR = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToCamBR);
      photonPoseEstimatorBL = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToCamBL);
    }
  
    /**
     * Gets the estimated robot pose from all cameras
     * 
     * @param prevEstimatedRobotPose
     * @return The estimated robot pose if targets are detected, otherwise {@code Optional.none()}
     */ 
    public Optional<EstimatedRobotPose> getPose(){
        PhotonPipelineResult cameraFRResult = cameraFR.getLatestResult();
        if(cameraFRResult.hasTargets() && cameraFRResult.getBestTarget().getPoseAmbiguity() < 0.05 && (m_autoMode == AutoMode.kOff || m_autoMode == AutoMode.kCoralStation)) {
          SmartDashboard.putString("Active Camera", "FR");
          return photonPoseEstimatorFR.update(cameraFRResult);
        }
        
        PhotonPipelineResult cameraFLResult = cameraFL.getLatestResult();
        if(cameraFLResult.hasTargets() && cameraFLResult.getBestTarget().getPoseAmbiguity() < 0.05 && (m_autoMode == AutoMode.kOff || m_autoMode == AutoMode.kReef)) {
          SmartDashboard.putString("Active Camera", "FL");
          
          latestFiducial = cameraFLResult.getBestTarget().fiducialId;
          latestDetected = System.nanoTime();

          return photonPoseEstimatorFL.update(cameraFLResult);
        }
    
        PhotonPipelineResult cameraBRResult = cameraBR.getLatestResult();
        if(cameraBRResult.hasTargets() && cameraBRResult.getBestTarget().getPoseAmbiguity() < 0.05 && m_autoMode == AutoMode.kOff) {
          SmartDashboard.putString("Active Camera", "BR");
          return photonPoseEstimatorBR.update(cameraBRResult);
        }
        
        PhotonPipelineResult cameraBLResult = cameraBL.getLatestResult();
        if(cameraBLResult.hasTargets() && cameraBLResult.getBestTarget().getPoseAmbiguity() < 0.05 && m_autoMode == AutoMode.kOff) {
          SmartDashboard.putString("Active Camera", "BL");
          return photonPoseEstimatorBL.update(cameraBLResult);
        }
      
        return Optional.empty();
    }
  
    public void setAutoMode(AutoMode autoMode){
      this.m_autoMode =  autoMode;
    }

    private AutoMode getAutoMode(){
      return this.m_autoMode;
    }

    // are you really reading this 
    // yeah of course i am smh

    /**
     * detects a target and returns a value as to whether it has been detected
     *
     * @return whether target is detected
     */
    public boolean detectsTarget() {
      // Query some boolean state, such as a digital sensor.
      boolean targetDetectedCameraFR = cameraFR.getLatestResult().hasTargets();// checks if camera(left) has detetected a target
      boolean targetDetectedCameraFL = cameraFL.getLatestResult().hasTargets();// checks if camera(Right) has detetected a target
      boolean targetDetectedCameraBR = cameraBR.getLatestResult().hasTargets();// checks if camera(Right) has detetected a target
      boolean targetDetectedCameraBL = cameraBL.getLatestResult().hasTargets();// checks if camera(left) has detetected a target
      boolean cameraA = targetDetectedCameraFR == true|targetDetectedCameraFL==true|targetDetectedCameraBR==true|targetDetectedCameraBL==true;//sets camera(all) to the combine value of both camera(left) and camera(right)
       
      return cameraA;//returns whether a target has been detetected
    }

    public int getLatestBfFiducial() {
      return this.latestFiducial;
    }

    public long getLatestBfDetected() {
      return this.latestDetected;
    }
  
  
    public record TargetInfo(double yaw, double pitch, double skew) {}
  
    /**
     * gets all the target information from left camera
     * 
     * @return all target info in a record
     */
    public TargetInfo targetIdentFR() {
      double yaw = cameraFR.getLatestResult().getBestTarget().getYaw(); //gets yaw from a april tag
      double pitch  = cameraFR.getLatestResult().getBestTarget().getPitch();//gets Pitch from a april tag
      double skew  = cameraFR.getLatestResult().getBestTarget().getSkew();//gets skew from a april tag
  
      return new TargetInfo(yaw, pitch, skew);
    }
  
     /**
     * gets all the target information from right camera
     * 
     * @return all target info in a record
     */
     public TargetInfo targetIdentFL() {
      double yaw = cameraFL.getLatestResult().getBestTarget().getYaw();//gets yaw from a april tag
      double pitch  = cameraFL.getLatestResult().getBestTarget().getPitch();//gets Pitch from a april tag
      double skew  = cameraFL.getLatestResult().getBestTarget().getSkew();//gets skew from a april tag
  
      return new TargetInfo(yaw, pitch, skew);
    }
  
    @Override
    public void periodic() {
      // This method will be called once per scheduler run
    //  SmartDashboard.putBoolean("Target Detected",  detectsTarget());//puts whether a target has been detected to the dashboard
     // SmartDashboard.putBoolean("Estimation", getPose().isPresent());      
      //TODO: error logging and alerting


    }
  
    @Override
    public void simulationPeriodic() {
      // This method will be called once per scheduler run during simulation
    }
  }
  