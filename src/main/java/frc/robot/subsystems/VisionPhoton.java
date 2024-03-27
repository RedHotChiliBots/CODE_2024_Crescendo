// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;
import java.util.Map;
import java.util.Optional;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
// import org.photonvision.simulation.PhotonCameraSim; //TODO Add Simulation
// import org.photonvision.simulation.SimCameraProperties;
// import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import frc.robot.Constants.VisionConstants;
import frc.utils.VisionUtils;

public class VisionPhoton extends SubsystemBase {
  /** Creates a new Vision. */

  // Change this to match the name of your camera
  private final PhotonCamera camera = new PhotonCamera(VisionConstants.kCameraName);

  private PhotonPoseEstimator photonEstimator = null;

  private double lastEstTimestamp = 0.0;

  // Simulation //TODO Add Simulation
  // private PhotonCameraSim cameraSim = null;
  // private VisionSystemSim visionSim = null;

  private PhotonPipelineResult result;
  private PhotonTrackedTarget target;
  private List<PhotonTrackedTarget> targets = null;
  private double range;
  private int targetId;

  private PIDController distController = new PIDController(VisionConstants.kDistP, VisionConstants.kDistI,
      VisionConstants.kDistD);
  private PIDController turnController = new PIDController(VisionConstants.kTurnP, VisionConstants.kTurnI,
      VisionConstants.kTurnD);

  private double forwardSpeed;
  private double rotationSpeed;

  private boolean insertOffset = false;

  private boolean hasTarget = false;
  private boolean rumble = false;

  // ==============================================================
  // Define Shuffleboard data

  private final ShuffleboardTab compTab = Shuffleboard.getTab("Competition");
  private final ShuffleboardTab visionTab = Shuffleboard.getTab("Vision");

  private final GenericEntry sbTargetID = visionTab.addPersistent("Target ID", 0.0)
      .withWidget("Text View").withPosition(0, 0).withSize(1, 1).getEntry();
  private final GenericEntry sbHasTargets = visionTab.addPersistent("Has Targets", false)
      .withWidget("Boolean Box").withPosition(0, 1).withSize(1, 1).getEntry();
  private final GenericEntry sbHasTarget = visionTab.addPersistent("Has Target", false)
      .withWidget("Boolean Box").withPosition(0, 2).withSize(1, 1).getEntry();
  private final GenericEntry sbDistAtTarget = visionTab.addPersistent("Dist At Target", false)
      .withWidget("Boolean Box").withPosition(0, 3).withSize(1, 1).getEntry();
  private final GenericEntry sbTurnAtTarget = visionTab.addPersistent("Turn At Target", false)
      .withWidget("Boolean Box").withPosition(0, 4).withSize(1, 1).getEntry();

  private final GenericEntry sbRange = visionTab.addPersistent("Range", 0)
      .withWidget("Text View").withPosition(2, 0).withSize(1, 1).getEntry();
  private final GenericEntry sbPitch = visionTab.addPersistent("Pitch", 0)
      .withWidget("Text View").withPosition(2, 1).withSize(1, 1).getEntry();
  private final GenericEntry sbYaw = visionTab.addPersistent("Yaw", 0)
      .withWidget("Text View").withPosition(2, 2).withSize(1, 1).getEntry();
  private final GenericEntry sbSkew = visionTab.addPersistent("Skew", 0)
      .withWidget("Text View").withPosition(2, 3).withSize(1, 1).getEntry();
  private final GenericEntry sbArea = visionTab.addPersistent("Area", 0)
      .withWidget("Text View").withPosition(2, 4).withSize(1, 1).getEntry();

  private final GenericEntry sbTgtX = visionTab.addPersistent("tgtX", 0)
      .withWidget("Text View").withPosition(3, 0).withSize(1, 1).getEntry();
  private final GenericEntry sbTgtY = visionTab.addPersistent("tgtY", 0)
      .withWidget("Text View").withPosition(3, 1).withSize(1, 1).getEntry();
  private final GenericEntry sbTgtZ = visionTab.addPersistent("tgtZ", 0)
      .withWidget("Text View").withPosition(3, 2).withSize(1, 1).getEntry();
  private final GenericEntry sbTgtRot = visionTab.addPersistent("tgtRot", 0)
      .withWidget("Text View").withPosition(3, 3).withSize(1, 1).getEntry();

  private final GenericEntry sbEstX = visionTab.addPersistent("estX", 0)
      .withWidget("Text View").withPosition(4, 0).withSize(1, 1).getEntry();
  private final GenericEntry sbEstY = visionTab.addPersistent("estY", 0)
      .withWidget("Text View").withPosition(4, 1).withSize(1, 1).getEntry();
  private final GenericEntry sbEstRot = visionTab.addPersistent("estRot", 0)
      .withWidget("Text View").withPosition(4, 3).withSize(1, 1).getEntry();

  private final GenericEntry sbERange = visionTab.addPersistent("estRange", 0)
      .withWidget("Text View").withPosition(5, 0).withSize(1, 1).getEntry();
  private final GenericEntry sbEAngle = visionTab.addPersistent("estAngle", 0)
      .withWidget("Text View").withPosition(5, 1).withSize(1, 1).getEntry();
  private final GenericEntry sbDeltaX = visionTab.addPersistent("estDeltaX", 0)
      .withWidget("Text View").withPosition(5, 2).withSize(1, 1).getEntry();
  private final GenericEntry sbDeltaY = visionTab.addPersistent("estDeltaY", 0)
      .withWidget("Text View").withPosition(5, 3).withSize(1, 1).getEntry();
  private final GenericEntry sbShootAngle = visionTab.addPersistent("Shoot Angle", 0)
      .withWidget("Text View").withPosition(5, 4).withSize(1, 1).getEntry();

  // Get information from target.
  double yaw = 0.0;
  double pitch = 0.0;
  double area = 0.0;
  double skew = 0.0;
  // Transform3d pose = null;
  // List<TargetCorner> corners = null;

  // // Get information from target.
  // int targetID = 0;
  // double poseAmbiguity = 0.0;
  // Transform3d bestCameraToTarget = null;
  // Transform3d alternateCameraToTarget = null;

  // double latencySeconds = 0.0;

  private Chassis chassis = null;

  private final NetworkTableInstance inst = NetworkTableInstance.getDefault();
  private final NetworkTable table = inst.getTable("limelight");

  private final NetworkTableEntry xEntry = table.getEntry("tx");
  private final NetworkTableEntry yEntry = table.getEntry("ty");
  private final NetworkTableEntry aEntry = table.getEntry("ta");
  private final NetworkTableEntry lEntry = table.getEntry("tl");
  private final NetworkTableEntry vEntry = table.getEntry("tv");
  private final NetworkTableEntry sEntry = table.getEntry("ts");

  private final NetworkTableEntry tshortEntry = table.getEntry("tshort");
  private final NetworkTableEntry tlongEntry = table.getEntry("tlong");
  private final NetworkTableEntry thorEntry = table.getEntry("thor");
  private final NetworkTableEntry tvertEntry = table.getEntry("tvert");
  private final NetworkTableEntry getpipeEntry = table.getEntry("getpipe");
  private final NetworkTableEntry camtranEntry = table.getEntry("camtran");
  private final NetworkTableEntry ledModeEntry = table.getEntry("ledMode");

  private double tx = 0.0;
  private double ty = 0.0;
  private double ta = 0.0;
  private double tl = 0.0;
  private double tv = 0.0;
  private double ts = 0.0;

  private double tshort = 0.0;
  private double tlong = 0.0;
  private double thor = 0.0;
  private double tvert = 0.0;
  private double getpipe = 0.0;
  private double[] camtran = {};
  double[] array = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
  private double ledMode = 0.0;

  public VisionPhoton(Chassis chassis) {
    System.out.println("+++++ Vision Constructor starting +++++");

    this.chassis = chassis;

    if (camera.getName() == VisionConstants.kCameraName) {
      System.out.println("Camera IS configured.");
    } else {
      DriverStation.reportError("Camera was not configured properly.", false);
    }

    photonEstimator = new PhotonPoseEstimator(VisionConstants.kTagLayout,
        PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camera, VisionConstants.kRobotToCam);

    photonEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

    if (photonEstimator.getRobotToCameraTransform().equals(VisionConstants.kRobotToCam)) {
      System.out.println("PhotonPoseEstimator IS configured.");
    } else {
      DriverStation.reportError("PhotonPoseEstimator was not configured properly.", false);
    }

    // ----- Simulation
    // if (Robot.isSimulation()) { // TODO Add Simulation
    // // Create the vision system simulation which handles cameras and targets on
    // the
    // // field.
    // visionSim = new VisionSystemSim("main");
    // // Add all the AprilTags inside the tag layout as visible targets to this
    // // simulated field.
    // visionSim.addAprilTags(VisionConstants.kTagLayout);
    // // Create simulated camera properties. These can be set to mimic your actual
    // // camera.
    // var cameraProp = new SimCameraProperties();
    // cameraProp.setCalibration(960, 720, Rotation2d.fromDegrees(90));
    // cameraProp.setCalibError(0.35, 0.10);
    // cameraProp.setFPS(15);
    // cameraProp.setAvgLatencyMs(50);
    // cameraProp.setLatencyStdDevMs(15);
    // // Create a PhotonCameraSim which will update the linked PhotonCamera's
    // values
    // // with visible
    // // targets.
    // cameraSim = new PhotonCameraSim(camera, cameraProp);
    // // Add the simulated camera to view the targets on this simulated field.
    // visionSim.addCamera(cameraSim, VisionConstants.kRobotToCam);

    // cameraSim.enableDrawWireframe(true);
    // }

    distController.setTolerance(0.1);
    turnController.setTolerance(0.5);

    compTab.addCamera("Camera", VisionConstants.kCameraName,
        "http://photonvision.local:1182/stream.mjpg")
        .withWidget("Camera Stream")
        .withPosition(7, 2).withSize(4, 4)
        .withProperties(Map.of("crosshaircolor", "Red", "showcontrols", false));

    // // Set driver mode to on.
    // camera.setDriverMode(true);

    // // Change pipeline to 2
    // camera.setPipelineIndex(2);

    Optional<EstimatedRobotPose> visionEst = getEstimatedGlobalPose();
    if (visionEst.isPresent()) {
      Pose2d estPose = visionEst.get().estimatedPose.toPose2d();
      chassis.resetPose(estPose);

      if (estPose.equals(chassis.getPose())) {
        System.out.println("Robot pose initialized.");
      } else {
        DriverStation.reportError("Robot pose was not initialized.", false);
      }
    } else {
      chassis.resetPose(new Pose2d(new Translation2d(0.0, 0.0), new Rotation2d(0.0)));
      DriverStation.reportError("Robot pose could not be determined.  Setting to ((0,0),0)", false);
    }

    setTargetId(15);

    System.out.println("----- Vision Constructor finished -----");
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    tx = xEntry.getDouble(0.0);
    ty = yEntry.getDouble(0.0);
    ta = aEntry.getDouble(0.0);
    tl = lEntry.getDouble(0.0);
    tv = vEntry.getDouble(0.0);
    ts = sEntry.getDouble(0.0);

    tshort = tshortEntry.getDouble(0.0);
    tlong = tlongEntry.getDouble(0.0);
    thor = thorEntry.getDouble(0.0);
    tvert = tvertEntry.getDouble(0.0);
    getpipe = getpipeEntry.getDouble(0.0);
    camtran = camtranEntry.getDoubleArray(array);
    ledMode = ledModeEntry.getDouble(0.0);

    // Correct pose estimate with vision measurements
    Optional<EstimatedRobotPose> visionEst = getEstimatedGlobalPose();

    // double[] visionPose = new double[] { visionEst.get().estimatedPose.getX(),
    // visionEst.get().estimatedPose.getY(),
    // Math.toDegrees(visionEst.get().estimatedPose.getRotation().getAngle()) };
    // SmartDashboard.putNumberArray("field/vision2", visionPose);

    visionEst.ifPresent(
        est -> {
          Pose2d estPose = est.estimatedPose.toPose2d();

          double[] visPose = new double[] { estPose.getX(), estPose.getY(), estPose.getRotation().getDegrees() };
          SmartDashboard.putNumberArray("field/vision", visPose);

          // Change our trust in the measurement based on the tags we can see
          Matrix<N3, N1> estStdDevs = getEstimationStdDevs(estPose);

          chassis.addVisionMeasurement(
              est.estimatedPose.toPose2d(), est.timestampSeconds, estStdDevs);
        });

    // Apply a random offset to pose estimator to test vision correction
    if (getInsertOffset()) {
      setInsertOffset(false);

      chassis.resetPose(VisionUtils.BumpPose(chassis.getPose()), false);
    }

    // Get information from target.
    result = getLatestResult();

    target = null;

    if (result.hasTargets()) {
      sbHasTargets.setBoolean(true);

      targets = result.getTargets();

      for (PhotonTrackedTarget t : targets) {
        if (t.getFiducialId() == targetId) {
          target = t;
          setHasTarget(true);
        } else {
          setHasTarget(false);
        }
      }

      sbHasTarget.setBoolean(hasTarget);

      if (hasTarget) {

        yaw = target.getYaw();
        pitch = target.getPitch();
        area = target.getArea();
        skew = target.getSkew();

        range = PhotonUtils.calculateDistanceToTargetMeters(
            VisionConstants.kCameraHeight,
            VisionConstants.kTargetHeight,
            VisionConstants.kCameraPitch,
            Units.degreesToRadians(pitch));

        sbRange.setDouble(Units.metersToFeet(range));

        sbYaw.setDouble(target.getYaw());
        sbPitch.setDouble(target.getPitch());
        sbArea.setDouble(target.getArea());
        sbSkew.setDouble(target.getSkew());
        sbTargetID.setDouble(target.getFiducialId());
        sbDistAtTarget.setBoolean(atDistTarget());
        sbTurnAtTarget.setBoolean(atTurnTarget());
      }
    } else {
      // If we have no targets, stay still.
      sbHasTargets.setBoolean(false);
      setHasTarget(false);
    }

    sbHasTarget.setBoolean(getHasTarget());
  }

  public double[] trackAprilTag() {

    if (hasTarget) {
      int tgtId = target.getFiducialId();
      Optional<Pose3d> tagPose = photonEstimator.getFieldTags().getTagPose(tgtId);
      double tgtX = tagPose.get().getTranslation().getX();
      double tgtY = tagPose.get().getTranslation().getY();
      double tgtZ = tagPose.get().getTranslation().getZ();
      double tgtRot = Math.toDegrees(tagPose.get().getRotation().getAngle());

      sbTgtX.setDouble(tgtX);
      sbTgtY.setDouble(tgtY);
      sbTgtZ.setDouble(tgtZ);
      sbTgtRot.setDouble(tgtRot);

      Pose2d estPose = chassis.getPose();
      double estX = estPose.getTranslation().getX();
      double estY = estPose.getTranslation().getY();
      double estRot = estPose.getRotation().getDegrees();

      sbEstX.setDouble(estX);
      sbEstY.setDouble(estY);
      sbEstRot.setDouble(estRot);

      double range = tagPose.get().toPose2d().getTranslation().getDistance(estPose.getTranslation());
      double angle = tagPose.get().toPose2d().getRotation().minus(estPose.getRotation()).getDegrees();
      double deltaX = tagPose.get().toPose2d().getTranslation().minus(estPose.getTranslation()).getX();
      double deltaY = tagPose.get().toPose2d().getTranslation().minus(estPose.getTranslation()).getY();

      sbERange.setDouble(range);
      sbEAngle.setDouble(angle);
      sbDeltaX.setDouble(deltaX);
      sbDeltaY.setDouble(deltaY);

      double shootAngle = Math.toDegrees(Math.sin((tgtZ - VisionConstants.kCameraHeight) / range));

      sbShootAngle.setDouble(shootAngle);

      double[] robotPose = new double[] { estPose.getX(), estPose.getY(), estPose.getRotation().getDegrees() };
      SmartDashboard.putNumberArray("field/robot", robotPose);

      double[] targetPose = new double[] { tagPose.get().getX(), tagPose.get().getY(), tagPose.get().getZ(),
          Math.toDegrees(tagPose.get().getRotation().getAngle()) };
      SmartDashboard.putNumberArray("field/target", targetPose);

      // Use this range as the measurement we give to the PID controller.
      // -1.0 required to ensure positive PID controller effort _increases_ range
      forwardSpeed = distController.calculate(range, VisionConstants.kTargetDist);

      // Also calculate angular power
      // -1.0 required to ensure positive PID controller effort _increases_ yaw
      rotationSpeed = -turnController.calculate(angle, 0);

    } else {
      forwardSpeed = 0;
      rotationSpeed = 0;
    }

    return new double[] { forwardSpeed, rotationSpeed };
  }

  public PhotonPipelineResult getLatestResult() {
    return camera.getLatestResult();
  }

  /**
   * The latest estimated robot pose on the field from vision data. This may be
   * empty. This should
   * only be called once per loop.
   *
   * @return An {@link EstimatedRobotPose} with an estimated pose, estimate
   *         timestamp, and targets
   *         used for estimation.
   */
  public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
    Optional<EstimatedRobotPose> visionEst = photonEstimator.update();
    double latestTimestamp = camera.getLatestResult().getTimestampSeconds();
    boolean newResult = Math.abs(latestTimestamp - lastEstTimestamp) > 1e-5;
    // if (Robot.isSimulation()) { // TODO Add Simulation
    // visionEst.ifPresentOrElse(
    // est -> getSimDebugField()
    // .getObject("VisionEstimation")
    // .setPose(est.estimatedPose.toPose2d()),
    // () -> {
    // if (newResult)
    // getSimDebugField().getObject("VisionEstimation").setPoses();
    // });
    // }
    if (newResult)
      lastEstTimestamp = latestTimestamp;
    return visionEst;
  }

  /**
   * The standard deviations of the estimated pose from
   * {@link #getEstimatedGlobalPose()}, for use
   * with {@link edu.wpi.first.math.estimator.SwerveDrivePoseEstimator
   * SwerveDrivePoseEstimator}.
   * This should only be used when there are targets visible.
   *
   * @param estimatedPose The estimated pose to guess standard deviations for.
   */
  public Matrix<N3, N1> getEstimationStdDevs(Pose2d estimatedPose) {
    var estStdDevs = VisionConstants.kSingleTagStdDevs;
    var targets = getLatestResult().getTargets();
    int numTags = 0;
    double avgDist = 0;
    for (var tgt : targets) {
      var tagPose = photonEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
      if (tagPose.isEmpty())
        continue;
      numTags++;
      avgDist += tagPose.get().toPose2d().getTranslation().getDistance(estimatedPose.getTranslation());
    }
    if (numTags == 0)
      return estStdDevs;
    avgDist /= numTags;
    // Decrease std devs if multiple targets are visible
    if (numTags > 1)
      estStdDevs = VisionConstants.kMultiTagStdDevs;
    // Increase std devs based on (average) distance
    if (numTags == 1 && avgDist > 4)
      estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
    else
      estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));

    return estStdDevs;
  }

  // ----- Simulation //TODO Add Simulation

  // public void simulationPeriodic(Pose2d robotSimPose) {
  // visionSim.update(robotSimPose);
  // }

  // /** Reset pose history of the robot in the vision system simulation. */
  // public void resetSimPose(Pose2d pose) {
  // if (Robot.isSimulation())
  // visionSim.resetRobotPose(pose);
  // }

  // /** A Field2d for visualizing our robot and objects on the field. */
  // public Field2d getSimDebugField() {
  // if (!Robot.isSimulation())
  // return null;
  // return visionSim.getDebugField();
  // }

  public void setTargetId(int id) {
    targetId = id;
  }

  public int getTargetId() {
    return targetId;
  }

  public boolean getHasTargets() {
    return result.hasTargets();
  }

  public boolean getHasTarget() {
    return hasTarget;
  }

  public void setHasTarget(boolean t) {
    hasTarget = t;
  }

  public boolean atDistTarget() {
    return distController.atSetpoint();
  }

  public boolean atTurnTarget() {
    return turnController.atSetpoint();
  }

  public double getRange() {
    return range;
  }

  public double getYaw() {
    return yaw;
  }

  public double getPitch() {
    return pitch;
  }

  public double getArea() {
    return area;
  }

  public double getSkew() {
    return skew;
  }

  public void setRumble(boolean r) {
    rumble = r;
  }

  public boolean getRumble() {
    return rumble;
  }

  public void toggleRumble() {
    rumble = !rumble;
  }

  public void setInsertOffset(boolean i) {
    insertOffset = i;
  }

  public boolean getInsertOffset() {
    return insertOffset;
  }
}
