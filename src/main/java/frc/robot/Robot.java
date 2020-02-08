package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;

public class Robot extends TimedRobot {

  public NTInterface ntInterface;
  public Drivetrain drivetrain;
  public NavX navX;
  public PID headingPID;
  public Joystick joy;

  public long pastTime = 0;
  public long currentTime = 0;
  public double navxHeading = 0;
  public double tarX = 0;
  public double tarY = 0;

  @Override
  public void robotInit() {
    ntInterface = new NTInterface();
    drivetrain = new Drivetrain();
    navX = new NavX();
    headingPID = new PID(0.001, 0, 0);
    joy = new Joystick(0);
  }
  
  @Override
  public void teleopInit() {
    pastTime = System.currentTimeMillis();
    currentTime = System.currentTimeMillis();
  }

  @Override
  public void teleopPeriodic() {
    currentTime = System.currentTimeMillis();
    navxHeading = navX.getConstrainedHeading();
    ntInterface.loop();
    
    double angleToTarget = Math.atan2(ntInterface.ty - tarY, ntInterface.tx - tarX);
    double turnPower = headingPID.update(angleToTarget, navxHeading, currentTime - pastTime);

    System.out.printf("turn: %.2f: , head: %.2f, angl: %.2f", turnPower, navxHeading, angleToTarget);

    if(joy.getRawButton(1)) {
      drivetrain.arcadeDrive(0, turnPower);
    } else {
      drivetrain.arcadeDrive(0, 0);
    }

    pastTime = currentTime;
  }
}
