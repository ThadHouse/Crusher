package first.robot;

import org.wpilib.command3.Scheduler;
import org.wpilib.command3.button.CommandGamepad;
import org.wpilib.driverstation.Gamepad;
import org.wpilib.opmode.PeriodicOpMode;
import org.wpilib.opmode.Teleop;

import first.robot.subsystems.DriveSubsystem;

@Teleop
public class DriveOpMode extends PeriodicOpMode {
    private final DriveSubsystem driveSubsystem;
    private final CommandGamepad gamepad;

    public DriveOpMode(Robot robot, CustomUserControls userControls) {
        driveSubsystem = robot.driveSubsystem;
        gamepad = userControls.driverGamepad;
    }

    @Override
    public void start() {
        var defaultCommand = driveSubsystem.runRepeatedly(this::doDrive).withPriority(0).named("Default Drive");
        Scheduler.getDefault().schedule(defaultCommand);
        gamepad.eastFace().whileTrue(driveSubsystem.runRepeatedly(() -> driveSubsystem.setX()).withPriority(1).named("Set X"));
        gamepad.leftBumper().risingEdge().onTrue(driveSubsystem.runRepeatedly(() -> driveSubsystem.zeroHeading()).withPriority(2).named("Zero Heading"));
    }

    private void doDrive() {
        double forward = -gamepad.getLeftX();
        double strafe = gamepad.getLeftY();
        double rotation = gamepad.getRightX();

        driveSubsystem.drive(forward, strafe, rotation, true);
    }
}
