
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
  
    PhotonPoseEstimator photonPoseEstimatorTF;
    PhotonPoseEstimator photonPoseEstimatorBF;
    PhotonPoseEstimator photonPoseEstimatorLB;
    PhotonPoseEstimator photonPoseEstimatorRB;
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
      TF,
      BF,
      LB,
      RB,
    }
  
    public static CameraSubsystem getInstance() {
      if (m_instance == null) {
        m_instance = new CameraSubsystem();
      }
  
      return m_instance;
    }
    
    PhotonCamera cameraTF;
    PhotonCamera cameraBF;
    PhotonCamera cameraLB;
    PhotonCamera cameraRB;

    /** Creates a new CameraSubsystem. */
    private CameraSubsystem() {
      super();
      // The field from AprilTagFields will be different depending on the game.
  
      // Gets the 2025 Welded AprilTag layout (change to AndyMark if not in the U.S.)
      AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
     
      //creating two cameras and assigning them a position relative to the center of the bot
      cameraTF = new PhotonCamera("Camera TF");
      Transform3d robotToCamTF = new Transform3d(
        new Translation3d(CameraConstants.topFrontX, CameraConstants.topFrontY , CameraConstants.topFrontZ),
        new Rotation3d(CameraConstants.topFrontRoll, CameraConstants.topFrontPitch, CameraConstants.topFrontYaw)
      ); //Cam mounted facing forward, upper left of the robot
      cameraBF = new PhotonCamera("Camera BF");
      Transform3d robotToCamBF = new Transform3d(
        new Translation3d(CameraConstants.bottomFrontX, CameraConstants.bottomFrontY, CameraConstants.bottomFrontZ),
        new Rotation3d(CameraConstants.bottomFrontRoll, CameraConstants.bottomFrontPitch, CameraConstants.bottomFrontYaw)
      ); //Cam mounted facing forward, in the bottom center of the robot
    
      //feeding in info for camera postion to photon pose estimator
      photonPoseEstimatorTF = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToCamTF);

      photonPoseEstimatorBF = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToCamBF);
 
    
      //creating two cameras and assigning them a position relative to the center of the bot
      cameraLB = new PhotonCamera("Camera LB");
      Transform3d robotToCamLB = new Transform3d(new Translation3d(CameraConstants.leftBackX, CameraConstants.leftBackY, CameraConstants.leftBackZ), new Rotation3d(CameraConstants.leftBackRoll, CameraConstants.leftBackPitch, CameraConstants.leftBackYaw)); //Cam mounted facing forward, half a meter forward of center, half a meter up from center.
      cameraRB = new PhotonCamera("Camera RB");
      Transform3d robotToCamRB = new Transform3d(new Translation3d(CameraConstants.rightBackX, CameraConstants.rightBackY, CameraConstants.rightBackZ), new Rotation3d(CameraConstants.rightBackRoll, CameraConstants.rightBackPitch, CameraConstants.rightBackYaw)); //Cam mounted facing forward, half a meter forward of center, half a meter up from center.
    
      //feeding in info for camera postion to photon pose estimator
      photonPoseEstimatorLB = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToCamLB);
      photonPoseEstimatorRB = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToCamRB);
    }
  
    /**
     * Gets the estimated robot pose from all cameras
     * 
     * @param prevEstimatedRobotPose
     * @return The estimated robot pose if targets are detected, otherwise {@code Optional.none()}
     */ 
    public Optional<EstimatedRobotPose> getPose(){
        PhotonPipelineResult cameraTfResult = cameraTF.getLatestResult();
        if(cameraTfResult.hasTargets() && cameraTfResult.getBestTarget().getPoseAmbiguity() < 0.05 && (m_autoMode == AutoMode.kOff || m_autoMode == AutoMode.kCoralStation)) {
          SmartDashboard.putString("Active Camera", "TF");
          return photonPoseEstimatorTF.update(cameraTfResult);
        }
        
        PhotonPipelineResult cameraBfResult = cameraBF.getLatestResult();
        if(cameraBfResult.hasTargets() && cameraBfResult.getBestTarget().getPoseAmbiguity() < 0.05 && (m_autoMode == AutoMode.kOff || m_autoMode == AutoMode.kReef)) {
          SmartDashboard.putString("Active Camera", "BF");
          
          latestFiducial = cameraBfResult.getBestTarget().fiducialId;
          latestDetected = System.nanoTime();

          return photonPoseEstimatorBF.update(cameraBfResult);
        }
    
        PhotonPipelineResult cameraLbResult = cameraLB.getLatestResult();
        if(cameraLbResult.hasTargets() && cameraLbResult.getBestTarget().getPoseAmbiguity() < 0.05 && m_autoMode == AutoMode.kOff) {
          SmartDashboard.putString("Active Camera", "LB");
          return photonPoseEstimatorLB.update(cameraLbResult);
        }
        
        PhotonPipelineResult cameraRbResult = cameraRB.getLatestResult();
        if(cameraRbResult.hasTargets() && cameraRbResult.getBestTarget().getPoseAmbiguity() < 0.05 && m_autoMode == AutoMode.kOff) {
          SmartDashboard.putString("Active Camera", "RB");
          return photonPoseEstimatorRB.update(cameraRbResult);
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
      boolean targetDetectedCameraTF = cameraTF.getLatestResult().hasTargets();// checks if camera(left) has detetected a target
      boolean targetDetectedCameraBF = cameraBF.getLatestResult().hasTargets();// checks if camera(Right) has detetected a target
      boolean targetDetectedCameraLB = cameraLB.getLatestResult().hasTargets();// checks if camera(left) has detetected a target
      boolean targetDetectedCameraRB = cameraRB.getLatestResult().hasTargets();// checks if camera(Right) has detetected a target
      boolean cameraA = targetDetectedCameraTF == true|targetDetectedCameraBF==true|targetDetectedCameraLB==true|targetDetectedCameraRB==true;//sets camera(all) to the combine value of both camera(left) and camera(right)
       
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
    public TargetInfo targetIdentTF() {
      double yaw = cameraTF.getLatestResult().getBestTarget().getYaw(); //gets yaw from a april tag
      double pitch  = cameraTF.getLatestResult().getBestTarget().getPitch();//gets Pitch from a april tag
      double skew  = cameraTF.getLatestResult().getBestTarget().getSkew();//gets skew from a april tag
  
      return new TargetInfo(yaw, pitch, skew);
    }
  
     /**
     * gets all the target information from right camera
     * 
     * @return all target info in a record
     */
     public TargetInfo targetIdentBF() {
      double yaw = cameraBF.getLatestResult().getBestTarget().getYaw();//gets yaw from a april tag
      double pitch  = cameraBF.getLatestResult().getBestTarget().getPitch();//gets Pitch from a april tag
      double skew  = cameraBF.getLatestResult().getBestTarget().getSkew();//gets skew from a april tag
  
      return new TargetInfo(yaw, pitch, skew);
    }
  
    @Override
    public void periodic() {
      // This method will be called once per scheduler run
      SmartDashboard.putBoolean("Target Detected",  detectsTarget());//puts whether a target has been detected to the dashboard
      SmartDashboard.putBoolean("Estimation", getPose().isPresent());      
      //TODO: error logging and alerting


    }
  
    @Override
    public void simulationPeriodic() {
      // This method will be called once per scheduler run during simulation
    }
  }
  