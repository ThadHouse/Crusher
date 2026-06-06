// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package first.robot;

import org.wpilib.command3.Scheduler;
import org.wpilib.driverstation.UserControlsInstance;
import org.wpilib.epilogue.Epilogue;
import org.wpilib.epilogue.Logged;
import org.wpilib.framework.OpModeRobot;
import first.robot.subsystems.DriveSubsystem;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
@Logged
@UserControlsInstance(CustomUserControls.class)
public class Robot extends OpModeRobot {
    @Logged
    public final DriveSubsystem driveSubsystem = new DriveSubsystem();

    @Override
    public void robotPeriodic() {
        Scheduler.getDefault().run();
            long start = System.nanoTime();
    var config = Epilogue.getConfig();
    Epilogue.first_robot_RobotLogger.tryUpdate(config.backend.getNested(config.root), this, config.errorHandler);
    config.backend.log("Epilogue/Stats/Last Run", (System.nanoTime() - start) / 1e6);
    }
}
