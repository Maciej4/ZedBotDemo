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

  public double offsetX = 0;
  public double offsetY = 0;

  @Override
  public void robotInit() {
    ntInterface = new NTInterface();
    drivetrain = new Drivetrain();
    navX = new NavX();
    headingPID = new PID(0.05, 0, 0);
    joy = new Joystick(0);
  }
  
  @Override
  public void teleopInit() {
    pastTime = System.currentTimeMillis();
    currentTime = System.currentTimeMillis();
    ntInterface.loop();

    // offsetX = ntInterface.tx;
    // offsetY = ntInterface.ty;
    drivetrain.arcadeDrive(0, 0);
  }

  @Override
  public void teleopPeriodic() {
    currentTime = System.currentTimeMillis();
    navxHeading = navX.getConstrainedHeading();
    ntInterface.loop();
    
    double angleToTarget = Math.toDegrees(Math.atan2(-ntInterface.tx, ntInterface.tz));
    double testAng = navxHeading - angleToTarget;
    //System.out.println("Gamer: " + normal(testAng));
    // double turnPower = headingPID.update(0, constrainAngle(-ntInterface.tx, -ntInterface.tz, navxHeading), currentTime - pastTime);
    double turnPower = headingPID.update(0, normal(testAng), currentTime - pastTime);
    turnPower = Math.max(Math.min(turnPower, 1.0), -1.0);

    System.out.printf("tn: %.2f, hd: %.2f, ag: %.2f, op:%.2f, tx: %.2f, tz: %.2f, gamer: %.2f\n", turnPower, navxHeading, angleToTarget, constrainAngle(-ntInterface.tx, -ntInterface.tz, navxHeading), ntInterface.tx, ntInterface.tz, normal(testAng));
    // System.out.println(constrainAngle(-ntInterface.tz, -ntInterface.tx, navxHeading));
    // System.`out.println("45TEST: " + constrainAngle(-1, 1, 0) + "-45TEST: " + constrainAngle(1, -1, 90) + " 150TEST: " + constrainAngle(1, 0.5, 0) + " 60TEST: " + constrainAngle(-1, -Math.sqrt(3), 0));
    if(joy.getRawButton(1)) {
      drivetrain.arcadeDrive(0, -turnPower);
    } else {
      drivetrain.arcadeDrive(-joy.getRawAxis(1), -joy.getRawAxis(4));
    }

    pastTime = currentTime;
  }

  private double constrainAngle(double x, double y, double navHead) {
    navHead = Math.toRadians(navHead);
    //changed to x, y in context of coord plane
    double dx = 0 - x;
    double dy = 0 - y;

    double num = dx * Math.cos(navHead) + dy * Math.sin(navHead);
    double mag = Math.sqrt(Math.pow(dx, 2) + Math.pow(dy, 2));
    double fin = orientation(x, navHead, y) * Math.toDegrees(Math.acos(num / mag));

    return fin;
    // return (angle - 180 + refHeading)%180;
  }

  private int orientation(double x, double navHead, double y) {
    if(x + Math.cos(navHead) < 0) {
      if(y < 0) {
        return 1;
      } else {
        return -1;
      }
    } else {
      if(y < 0) {
        return -1;
      } else {
        return 1;
      }
    }
  }

  public double normal(double heading) {
    // Constrains between -360 and 360
    heading %= 360; 
        
    // Constrains between -180 and 180
    if (heading >= 180) {
        heading -= 360;
    } else if (heading < -180) {
        heading += 360;
    }

    return heading;
  }
}
