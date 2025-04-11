package frc.robot.commands.head;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.HeadConstants;
import frc.robot.subsystems.ElevSubsystem;
import frc.robot.subsystems.HeadSubsystem;

public class ShootSlow extends Command {

  private final HeadSubsystem head;
  private Timer timeout;

  public ShootSlow(HeadSubsystem headSub, ElevSubsystem elevSub) {
    this.head = headSub;
    addRequirements(head);
  }

  @Override
  public void initialize() {
    timeout = new Timer();

    head.flywheelLeft(HeadConstants.shootSlowVBus);
    head.flywheelRight(HeadConstants.shootSlowVBus);
  }

  @Override
  public void execute() {
    if (!head.getHasCoral() && !timeout.isRunning()) {
      timeout.start();
    }
  }

  @Override
  public void end(boolean interrupted) {
    head.stop();
  }

  @Override
  public boolean isFinished() {
    return timeout.hasElapsed(HeadConstants.shootTime);
  }
}
