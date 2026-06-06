package first.robot;

import org.wpilib.command3.Command;
import org.wpilib.command3.Scheduler;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.opmode.Autonomous;
import org.wpilib.opmode.PeriodicOpMode;

import first.robot.Constants.AutoConstants;
import first.robot.subsystems.DriveSubsystem;

@Autonomous(name = "Forward 1 Meter")
public class ForwardOneMeterAutoOpMode extends PeriodicOpMode {
    private static final double kTargetDistanceMeters = 1.0;

    private final DriveSubsystem driveSubsystem;
    private Command autoDriveCommand;

    public ForwardOneMeterAutoOpMode(Robot robot) {
        driveSubsystem = robot.driveSubsystem;
    }

    @Override
    public void start() {
        driveSubsystem.resetOdometry(Pose2d.kZero);

        autoDriveCommand =
                driveSubsystem
                        .run(coroutine -> {
                            while (!hasDrivenTargetDistance()) {
                                driveSubsystem.driveMetersPerSecond(
                                        AutoConstants.kMaxSpeedMetersPerSecond, 0, 0, false);
                                coroutine.yield();
                            }

                            driveSubsystem.setX();
                        })
                        .whenCanceled(() -> driveSubsystem.setX())
                        .named("Forward 1 Meter");
        Scheduler.getDefault().schedule(autoDriveCommand);
    }

    @Override
    public void periodic() {}

    private boolean hasDrivenTargetDistance() {
        return driveSubsystem.getPose().getX() >= kTargetDistanceMeters;
    }

    @Override
    public void end() {
        if (autoDriveCommand != null) {
            Scheduler.getDefault().cancel(autoDriveCommand);
        }
        driveSubsystem.setX();
    }
}
