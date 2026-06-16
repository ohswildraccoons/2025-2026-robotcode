
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
  
    PhotonPoseEstimator photonPoseEstimatorFL;
    PhotonPoseEstimator photonPoseEstimatorFR;
    PhotonPoseEstimator photonPoseEstimatorBL;
    PhotonPoseEstimator photonPoseEstimatorBR;
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
    
    PhotonCamera cameraFL;
    PhotonCamera cameraFR;
    PhotonCamera cameraBL;
    PhotonCamera cameraBR;

    /** Creates a new CameraSubsystem. */
    private CameraSubsystem() {
      super();
      // The field from AprilTagFields will be different depending on the game.
  
      // Gets the 2025 Welded AprilTag layout (change to AndyMark if not in the U.S.)
      AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);
     
      //creating two cameras and assigning them a position relative to the center of the bot
      cameraFL = new PhotonCamera("FL_Cam");
            Transform3d robotToCamFL = new Transform3d(
        new Translation3d(CameraConstants.frontLeftX, CameraConstants.frontLeftY, CameraConstants.frontLeftZ),
        new Rotation3d(CameraConstants.frontLeftRoll, CameraConstants.frontLeftPitch, CameraConstants.frontLeftYaw)
      ); //Cam mounted facing forward, upper right of the robot
      
      cameraFR = new PhotonCamera("FR_Cam");
      Transform3d robotToCamFR = new Transform3d(
        new Translation3d(CameraConstants.frontRightX, CameraConstants.frontRightY , CameraConstants.frontRightZ),
        new Rotation3d(CameraConstants.frontRightRoll, CameraConstants.frontRightPitch, CameraConstants.frontRightYaw)
      ); //Cam mounted facing forward, upper left of the robot

    
      //feeding in info for camera postion to photon pose estimator
      photonPoseEstimatorFL = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToCamFL);

      photonPoseEstimatorFR = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToCamFR);
 
    
      //creating two cameras and assigning them a position relative to the center of the bot
      cameraBL = new PhotonCamera("BL_Cam");
      Transform3d robotToCamBL = new Transform3d(
        new Translation3d(CameraConstants.backRightX, CameraConstants.backRightY, CameraConstants.backRightZ),
        new Rotation3d(CameraConstants.backRightRoll, CameraConstants.backRightPitch, CameraConstants.backRightYaw)
      ); //Cam mounted facing forward, lower left of the robot
      cameraBR = new PhotonCamera("BR_Cam");
      Transform3d robotToCamBR = new Transform3d(
        new Translation3d(CameraConstants.backLeftX, CameraConstants.backLeftY, CameraConstants.backLeftZ),
        new Rotation3d(CameraConstants.backLeftRoll, CameraConstants.backLeftPitch, CameraConstants.backLeftYaw)); //Cam mounted facing forward, half a meter forward of center, half a meter up from center.
    
      //feeding in info for camera postion to photon pose estimator
      photonPoseEstimatorBL = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToCamBL);
      photonPoseEstimatorBR = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToCamBR);
    }
  
    /**
     * Gets the estimated robot pose from all cameras
     * 
     * @param prevEstimatedRobotPose
     * @return The estimated robot pose if targets are detected, otherwise {@code Optional.none()}
     */   
    public Optional<EstimatedRobotPose> getPose(Pose2d currentPose) {
        // Set reference pose for all estimators
        photonPoseEstimatorFL.setReferencePose(currentPose);
        photonPoseEstimatorFR.setReferencePose(currentPose);
        photonPoseEstimatorBL.setReferencePose(currentPose);
        photonPoseEstimatorBR.setReferencePose(currentPose);

        // Try each camera and return the first valid pose
        PhotonPipelineResult rFL = cameraFL.getLatestResult();
        if (rFL.hasTargets()){
                      SmartDashboard.putNumber("April tag id", rFL.getBestTarget().fiducialId);
            return photonPoseEstimatorFL.update(rFL);

        }
        PhotonPipelineResult rFR = cameraFR.getLatestResult();
        if (rFR.hasTargets())
            return photonPoseEstimatorFR.update(rFR);

        PhotonPipelineResult rBL = cameraBL.getLatestResult();
        if (rBL.hasTargets())
            return photonPoseEstimatorBL.update(rBL);

        PhotonPipelineResult rBR = cameraBR.getLatestResult();
        if (rBR.hasTargets())
            return photonPoseEstimatorBR.update(rBR);

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
      boolean targetDetectedCameraFL = cameraFL.getLatestResult().hasTargets();// checks if camera(left) has detetected a target
      boolean targetDetectedCameraFR = cameraFR.getLatestResult().hasTargets();// checks if camera(Right) has detetected a target
      boolean targetDetectedCameraBL = cameraBL.getLatestResult().hasTargets();// checks if camera(left) has detetected a target
      boolean targetDetectedCameraBR = cameraBR.getLatestResult().hasTargets();// checks if camera(Right) has detetected a target
      boolean cameraA = targetDetectedCameraFL == true|targetDetectedCameraFR==true|targetDetectedCameraBL==true|targetDetectedCameraBR==true;//sets camera(all) to the combine value of both camera(left) and camera(right)
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
    public TargetInfo targetIdentFL() {
      double yaw = cameraFL.getLatestResult().getBestTarget().getYaw(); //gets yaw from a april tag
      double pitch  = cameraFL.getLatestResult().getBestTarget().getPitch();//gets Pitch from a april tag
      double skew  = cameraFL.getLatestResult().getBestTarget().getSkew();//gets skew from a april tag
  
      return new TargetInfo(yaw, pitch, skew);
    }
  
     /**
     * gets all the target information from right camera
     * 
     * @return all target info in a record
     */
     public TargetInfo targetIdentBL() {
      double yaw = cameraBL.getLatestResult().getBestTarget().getYaw();//gets yaw from a april tag
      double pitch  = cameraBL.getLatestResult().getBestTarget().getPitch();//gets Pitch from a april tag
      double skew  = cameraBL.getLatestResult().getBestTarget().getSkew();//gets skew from a april tag
  
      return new TargetInfo(yaw, pitch, skew);
    }
  
    @Override
    public void periodic() {
      // This method will be called once per scheduler run
      SmartDashboard.putBoolean("Target Detected",  detectsTarget());//puts whether a target has been detected to the dashboard
      SmartDashboard.putBoolean("Est.imation", getPose(previPose2d).isPresent());      
      //TODO: error logging and alerting


    }
  
    @Override
    public void simulationPeriodic() {
      
    }
  }
  