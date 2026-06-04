// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package first.robot;

import org.wpilib.driverstation.DefaultUserControls;
import org.wpilib.driverstation.UserControlsInstance;
import org.wpilib.framework.OpModeRobot;
import org.wpilib.hardware.bus.I2C.Port;

import first.robot.subsystems.DriveSubsystem;
import first.robot.subsystems.GoBildaPinpoint;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
// @Logged
@UserControlsInstance(DefaultUserControls.class)
public class Robot extends OpModeRobot {
    public final DriveSubsystem driveSubsystem = new DriveSubsystem();
}
