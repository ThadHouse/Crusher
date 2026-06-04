package first.robot;

import org.wpilib.command3.button.CommandGamepad;
import org.wpilib.driverstation.UserControls;

public class CustomUserControls implements UserControls {
  public final CommandGamepad driverGamepad = new CommandGamepad(0);
}
