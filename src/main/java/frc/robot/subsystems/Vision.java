// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Map;
import java.util.Optional;

// import org.photonvision.PhotonCamera;
// import org.photonvision.PhotonPoseEstimator;
// import org.photonvision.PhotonPoseEstimator.PoseStrategy;
// import org.photonvision.PhotonUtils;
// import org.photonvision.targeting.PhotonPipelineResult;
// import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.VisionConstants;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;

public class Vision extends SubsystemBase {
        /** Creates a new Vision. */

        // Simulation //TODO Add Simulation
        // private PhotonCameraSim cameraSim = null;
        // private VisionSystemSim visionSim = null;

        private LimelightHelpers.PoseEstimate limelightMeasurement = null;
        private LimelightHelpers.LimelightResults llResults = null;

        private double range;
        private int targetId;

        private PIDController distController = new PIDController(VisionConstants.kDistP, VisionConstants.kDistI,
                        VisionConstants.kDistD);
        private PIDController turnController = new PIDController(VisionConstants.kTurnP, VisionConstants.kTurnI,
                        VisionConstants.kTurnD);

        private double forwardSpeed;
        private double rotationSpeed;

        private boolean insertOffset = false;

        // private boolean hasTarget = false;
        LimelightHelpers.LimelightTarget_Fiducial trackingFiducial = null;

        private boolean rumble = false;

        private Optional<Alliance> alliance = null;

        private Chassis chassis = null;

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

        private final GenericEntry sbBotX = visionTab.addPersistent("estX", 0)
                        .withWidget("Text View").withPosition(4, 0).withSize(1, 1).getEntry();
        private final GenericEntry sbBotY = visionTab.addPersistent("estY", 0)
                        .withWidget("Text View").withPosition(4, 1).withSize(1, 1).getEntry();
        private final GenericEntry sbBotZ = visionTab.addPersistent("estZ", 0)
                        .withWidget("Text View").withPosition(4, 2).withSize(1, 1).getEntry();
        private final GenericEntry sbBotRot = visionTab.addPersistent("estRot", 0)
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

        public Vision() {
                System.out.println("+++++ Vision Constructor starting +++++");

                distController.setTolerance(0.1);
                turnController.setTolerance(0.5);

                compTab.addCamera("Camera", VisionConstants.kCameraName, "http://10.44.53.11:5800")
                                .withWidget("Camera Stream")
                                .withPosition(7, 2).withSize(4, 4)
                                .withProperties(Map.of("crosshaircolor", "Red", "showcontrols", false));

                // // Set driver mode to on.
                // camera.setDriverMode(true);

                // // Change pipeline to 2
                // camera.setPipelineIndex(2);

                // Optional<EstimatedRobotPose> visionEst = getEstimatedGlobalPose();
                // if (visionEst.isPresent()) {
                // Pose2d estPose = visionEst.get().estimatedPose.toPose2d();
                // chassis.resetPose(estPose);

                // if (estPose.equals(chassis.getPose())) {
                // System.out.println("Robot pose initialized.");
                // } else {
                // DriverStation.reportError("Robot pose was not initialized.", false);
                // }
                // } else {
                // chassis.resetPose(new Pose2d(new Translation2d(0.0, 0.0), new
                // Rotation2d(0.0)));
                // DriverStation.reportError("Robot pose could not be determined. Setting to
                // ((0,0),0)", false);
                // }

                alliance = DriverStation.getAlliance();
                if (alliance.isPresent()) {
                        System.out.println("Alliance: " + alliance.get().toString());
                } else {
                        System.out.println("Alliance is not set");
                        // alliance = DriverStation.Alliance.Blue;
                }

                setTargetId(15);

                System.out.println("----- Vision Constructor finished -----");
        }

        @Override
        public void periodic() {

                llResults = LimelightHelpers.getLatestResults("limelight");

                LimelightHelpers.LimelightTarget_Fiducial[] llFiducials = llResults.targetingResults.targets_Fiducials;
                trackingFiducial = null;
                for (LimelightHelpers.LimelightTarget_Fiducial fiducial : llFiducials) {
                        if (fiducial.fiducialID == getTargetId()) {
                                trackingFiducial = fiducial;
                                break;
                        }
                }

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
        }

        public void setChassis(Chassis chassis) {
                System.out.println("Vision: Chassis initialized");
                this.chassis = chassis;
        }

        public boolean getHasTargets() {
                return limelightMeasurement.tagCount > 0;
        }

        public void updateLimeLightPose() {
                if (alliance.isPresent()) {
                        if (alliance.get() == Alliance.Blue) {
                                limelightMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");
                        } else if (alliance.get() == Alliance.Red) {
                                limelightMeasurement = LimelightHelpers.getBotPoseEstimate_wpiRed("limelight");
                        } else {
                                DriverStation.reportError("Alliance not set.", false);
                        }
                }

                limelightMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");

                updatePoseEstimatorWithVisionBotPose(limelightMeasurement);
        }

        public void updatePoseEstimatorWithVisionBotPose(PoseEstimate limelightMeasurement) {

                // No chassis subsystem
                if (chassis == null) {
                        System.out.println("Chassis not configured");
                        return;
                }

                // invalid LL data
                if (limelightMeasurement.pose.getX() == 0.0) {
                        return;
                }

                // distance from current pose to vision estimated pose
                double poseDifference = chassis.poseEstimator.getEstimatedPosition().getTranslation()
                                .getDistance(limelightMeasurement.pose.getTranslation());

                if (limelightMeasurement.tagCount >= 0) {
                        double xyStds;
                        double degStds;
                        // multiple targets detected
                        if (limelightMeasurement.tagCount >= 2) {
                                xyStds = 0.5;
                                degStds = 6;
                        }
                        // 1 target with large area and close to estimated pose
                        else if (limelightMeasurement.avgTagArea > 0.8 && poseDifference < 0.5) {
                                xyStds = 1.0;
                                degStds = 12;
                        }
                        // 1 target farther away and estimated pose is close
                        else if (limelightMeasurement.avgTagArea > 0.1 && poseDifference < 0.3) {
                                xyStds = 2.0;
                                degStds = 30;
                        }
                        // conditions don't match to add a vision measurement
                        else {
                                return;
                        }

                        chassis.poseEstimator.setVisionMeasurementStdDevs(
                                        VecBuilder.fill(xyStds, xyStds, Units.degreesToRadians(degStds)));
                        chassis.poseEstimator.addVisionMeasurement(limelightMeasurement.pose,
                                        Timer.getFPGATimestamp() - limelightMeasurement.latency / 1000.0);
                }
        }

        public double[] trackAprilTag() {

                if (trackingFiducial != null) {

                        Pose3d tgtPose = trackingFiducial.getCameraPose_TargetSpace();
                        Pose3d botPose = trackingFiducial.getRobotPose_TargetSpace();

                        double tgtId = trackingFiducial.fiducialID;

                        double tgtX = tgtPose.getX();
                        double tgtY = tgtPose.getY();
                        double tgtZ = tgtPose.getZ();
                        double tgtRot = tgtPose.getRotation().getAngle(); // radians

                        sbTgtX.setDouble(tgtX);
                        sbTgtY.setDouble(tgtY);
                        sbTgtZ.setDouble(tgtZ);
                        sbTgtRot.setDouble(tgtRot);

                        // Pose2d estPose = chassis.getPose();
                        double botX = botPose.getTranslation().getX();
                        double botY = botPose.getTranslation().getY();
                        double botZ = botPose.getTranslation().getZ();
                        double botRot = botPose.getRotation().getAngle();

                        sbBotX.setDouble(botX);
                        sbBotY.setDouble(botY);
                        sbBotZ.setDouble(botZ);
                        sbBotRot.setDouble(botRot);

                        double angle = botPose.toPose2d().getRotation().minus(tgtPose.toPose2d().getRotation())
                                        .getDegrees();

                        double deltaX = botPose.toPose2d().getTranslation().minus(tgtPose.toPose2d().getTranslation())
                                        .getX();
                        double deltaY = botPose.toPose2d().getTranslation().minus(tgtPose.toPose2d().getTranslation())
                                        .getY();
                        double range = Math.sqrt(Math.pow(deltaX, 2) + Math.pow(deltaY, 2));

                        sbERange.setDouble(range);
                        sbEAngle.setDouble(angle);
                        sbDeltaX.setDouble(deltaX);
                        sbDeltaY.setDouble(deltaY);

                        double shootAngle = Math.toDegrees(Math.sin((tgtZ -
                                        VisionConstants.kCameraHeight) / range));

                        sbShootAngle.setDouble(shootAngle);

                        double[] robotPose = new double[] { botPose.getX(), botPose.getY(),
                                        botPose.getRotation().getAngle() };
                        SmartDashboard.putNumberArray("field/robot", robotPose);

                        double[] targetPose = new double[] { tgtPose.getX(),
                                        tgtPose.getY(), tgtPose.getZ(),
                                        Math.toDegrees(tgtPose.getRotation().getAngle()) };
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

        public void setTargetId(int id) {
                targetId = id;
        }

        public int getTargetId() {
                return targetId;
        }

        // public boolean getHasTargets() {
        // return result.hasTargets();
        // }

        // public boolean getHasTarget() {
        // return hasTarget;
        // }

        // public void setHasTarget(boolean t) {
        // hasTarget = t;
        // }

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

        // public void setInsertOffset(boolean i) {
        // insertOffset = i;
        // }

        // public boolean getInsertOffset() {
        // return insertOffset;
        // }
}
