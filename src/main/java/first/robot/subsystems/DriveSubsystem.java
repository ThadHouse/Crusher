// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package first.robot.subsystems;

import static org.wpilib.units.Units.RadiansPerSecond;

// import static edu.wpi.first.units.Units.RadiansPerSecond;

import org.wpilib.command3.Mechanism;
import org.wpilib.command3.Scheduler;
import org.wpilib.hardware.bus.I2C.Port;
import org.wpilib.hardware.imu.OnboardIMU;
import org.wpilib.hardware.imu.OnboardIMU.MountOrientation;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.kinematics.SwerveDriveKinematics;
import org.wpilib.math.kinematics.SwerveDriveOdometry;
import org.wpilib.math.kinematics.SwerveModulePosition;
import org.wpilib.math.kinematics.SwerveModuleVelocity;

import first.robot.Constants.DriveConstants;

// @Logged
public class DriveSubsystem extends Mechanism {
  // Create MAXSwerveModules
  private final MAXSwerveModule m_frontLeft = new MAXSwerveModule(
      DriveConstants.kLeftSparkFlexCanBus,
      DriveConstants.kFrontLeftDrivingCanId,
      DriveConstants.kFrontLeftTurningCanId,
      DriveConstants.kFrontLeftChassisAngularOffset);

  private final MAXSwerveModule m_frontRight = new MAXSwerveModule(
      DriveConstants.kRightSparkFlexCanBus,
      DriveConstants.kFrontRightDrivingCanId,
      DriveConstants.kFrontRightTurningCanId,
      DriveConstants.kFrontRightChassisAngularOffset);

  private final MAXSwerveModule m_rearLeft = new MAXSwerveModule(
      DriveConstants.kLeftSparkFlexCanBus,
      DriveConstants.kRearLeftDrivingCanId,
      DriveConstants.kRearLeftTurningCanId,
      DriveConstants.kBackLeftChassisAngularOffset);

  private final MAXSwerveModule m_rearRight = new MAXSwerveModule(
      DriveConstants.kRightSparkFlexCanBus,
      DriveConstants.kRearRightDrivingCanId,
      DriveConstants.kRearRightTurningCanId,
      DriveConstants.kBackRightChassisAngularOffset);

  private final OnboardIMU m_onboardImu = new OnboardIMU(MountOrientation.FLAT);

  // Odometry class for tracking the robot's pose
  private final SwerveDriveOdometry m_odometry;

  // Create a new DriveSubsystem
  public DriveSubsystem() {
    m_pinpoint.resetHeading();
    m_onboardImu.resetYaw();
    m_frontLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearLeft.resetEncoders();
    m_rearRight.resetEncoders();
    m_odometry = new SwerveDriveOdometry(
      DriveConstants.kDriveKinematics,
      m_onboardImu.getRotation2d(),
      new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
      });

    Scheduler.getDefault().addPeriodic(this::periodic);
  }

  // Update odometry in the periodic block
  public void periodic() {
    m_pinpoint.update();
    m_odometry.update(
        m_onboardImu.getRotation2d(),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        });
  }

  // Return the currently-estimated pose of the robot
  public Pose2d getPose() {
    return m_odometry.getPose();
  }

  // Reset the odometry to the specified pose
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(
        m_onboardImu.getRotation2d(),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        },
        pose);
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    // Convert the commanded speeds into the correct units for the drivetrain
    double xSpeedDelivered = xSpeed * DriveConstants.kMaxSpeedMetersPerSecond;
    double ySpeedDelivered = ySpeed * DriveConstants.kMaxSpeedMetersPerSecond;
    double rotDelivered = rot * DriveConstants.kMaxAngularSpeed;

    var chassisSpeeds = new ChassisVelocities(xSpeedDelivered, ySpeedDelivered, rotDelivered);
    if (fieldRelative) {
      chassisSpeeds = chassisSpeeds.toRobotRelative(m_odometry.getPose().getRotation());
    }
    chassisSpeeds = chassisSpeeds.discretize(0.02);
    var swerveModuleStates = DriveConstants.kDriveKinematics.toWheelVelocities(chassisSpeeds);
    var newStates = SwerveDriveKinematics.desaturateWheelVelocities(
        swerveModuleStates, DriveConstants.kMaxWheelSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(newStates[0]);
    m_frontRight.setDesiredState(newStates[1]);
    m_rearLeft.setDesiredState(newStates[2]);
    m_rearRight.setDesiredState(newStates[3]);
  }

  // Set the wheels into an X formation to prevent movement
  public void setX() {
    m_frontLeft.setDesiredState(new SwerveModuleVelocity(0, Rotation2d.fromDegrees(45)));
    m_frontRight.setDesiredState(new SwerveModuleVelocity(0, Rotation2d.fromDegrees(-45)));
    m_rearLeft.setDesiredState(new SwerveModuleVelocity(0, Rotation2d.fromDegrees(-45)));
    m_rearRight.setDesiredState(new SwerveModuleVelocity(0, Rotation2d.fromDegrees(45)));
  }

  // Set the swerve module states
  public void setModuleStates(SwerveModuleVelocity[] desiredStates) {
    var newStates = SwerveDriveKinematics.desaturateWheelVelocities(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(newStates[0]);
    m_frontRight.setDesiredState(newStates[1]);
    m_rearLeft.setDesiredState(newStates[2]);
    m_rearRight.setDesiredState(newStates[3]);
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in radians
   */
  public double getHeading() {
    return m_odometry.getPose().getRotation().getRadians();
  }

  /**
   * Returns the heading of the robot from the IMU directly.
   *
   * @return the imu's heading in radians
   */
  public double getRawHeading() {
    return m_onboardImu.getYawRadians();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return m_onboardImu.getGyroRateZ();
  }

  private final GoBildaPinpoint m_pinpoint = new GoBildaPinpoint(Port.PORT_0);

  public double getPinpointTurnRate() {
    return m_pinpoint.getHeadingVelocity().in(RadiansPerSecond);
  }

  public double getPinpointHeading() {
    return m_pinpoint.getHeading().getRadians();
  }
}
