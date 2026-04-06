package frc.robot.Commands;

import org.photonvision.PhotonCamera;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;

public class AlignToTagCommand extends Command {
    //Tag IDs to align to - add or change as needed
    private static final int[] TARGET_IDS = {10, 26, 18, 21, 11, 8};

    //How close you want to stop from the tag(meters)
    private static final double FRONT_LEFT_CAMERA_YAW_OFFSET = 10;
    private static final double FRONT_RIGHT_CAMERA_YAW_OFFSET = -10;
        

    //PID for yaw correction (turning to face the tag)
    //Tune kP:increase if turning is too slow, decrease if it oscillates
    private final PIDController yawController = new PIDController(.2, 0, 0);

    private final DriveSubsystem drivetrain;
    private final PhotonCamera frontRightCamera;
    private final PhotonCamera frontLeftCamera;
    private final AprilTagFieldLayout fieldLayout;
    
    public AlignToTagCommand(DriveSubsystem drivetrain, PhotonCamera frontLeftCamera, PhotonCamera frontRightCamera, AprilTagFieldLayout fieldLayout) {
        this.drivetrain = drivetrain;
        this.frontLeftCamera = frontLeftCamera;
        this.frontRightCamera = frontRightCamera;
        this.fieldLayout = fieldLayout;
        addRequirements(drivetrain);

        yawController.setTolerance(5); //Degrees
    }

    @Override
    public void initialize() {
        yawController.reset();
    }

    @Override
    public void execute() {
        CameraTarget found = findTarget(frontLeftCamera);
        if (found == null) found = findTarget(frontRightCamera);

        if (found == null) {
            stop();
            SmartDashboard.putBoolean("Align/Target Visible", false);
            SmartDashboard.putString("Align/Camera used", "none");
            return;
        }

        SmartDashboard.putBoolean("Align/Target Visible", true);
        SmartDashboard.putNumber("Align/Matched Tag ID",  found.id);
        SmartDashboard.putString("Align/Camera Used",     found.cameraName);

        double turn = yawController.calculate(found.yaw, 0);
        turn = Math.max(-.5, Math.min(.5, turn));

        //Calculate drive speeds from field pose
        double xSpeed = 0;
        double ySpeed = 0;

        var tagPose = fieldLayout.getTagPose(found.id);
        if (tagPose.isPresent()) {
            Pose2d robotPose = drivetrain.getPose();

        }    }

    @Override
    public void end(boolean interrupted) {
        stop();
        yawController.reset();
    }

    @Override
    public boolean isFinished(){
        return false;
    }
    private void stop() {
        drivetrain.drive(0, 0, 0, true);
    }

    private CameraTarget findTarget(PhotonCamera camera) {
        var result = camera.getLatestResult();
        if (!result.hasTargets()) return null;

        double yawOffset = camera.getName().equals(Constants.Vision.kFrontLeftCameraName)
            ? FRONT_LEFT_CAMERA_YAW_OFFSET
            : FRONT_RIGHT_CAMERA_YAW_OFFSET;

        for (var target : result.getTargets()) {
            for (int id : TARGET_IDS) {
                if (target.getFiducialId() == id) {
                    return new CameraTarget(id, target.getYaw() - yawOffset, camera.getName());
                }
            }
        }
        return null;
    }

    private static class CameraTarget {
        final int id;
        final double yaw;
        final String cameraName;

        CameraTarget(int id, double yaw, String cameraName) {
            this.id = id;
            this.yaw = yaw;
            this.cameraName = cameraName;
        }
    }
}
