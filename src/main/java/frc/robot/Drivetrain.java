package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
public class Drivetrain
{
    public final double speedFac = -1, turnFac = -1;
    public static final double driveClickToCm = 2*Math.PI;

    public double startPosLeft = 0, startPosRight = 0;
    public double pastLeftDist = 0, pastRightDist = 0;
    public long pastTime;

    public CANSparkMax leftMotor1, leftMotor2, rightMotor1, rightMotor2;

    public PID leftDrivePID = new PID(0.09, 0, 0);
    public PID rightDrivePID = new PID(0.09, 0, 0);
    
    public Drivetrain ()
    {
        leftMotor1 = new CANSparkMax(1, MotorType.kBrushless);
        leftMotor2 = new CANSparkMax(2, MotorType.kBrushless);
        rightMotor1 = new CANSparkMax(3, MotorType.kBrushless);
        rightMotor2 = new CANSparkMax(4, MotorType.kBrushless);

        pastTime = System.currentTimeMillis();
    }

    public void arcadeDrive (double power, double turn) {
        power *= speedFac;
        turn *=  turnFac;
        double leftPower = turn - power;
        double rightPower = turn + power;

        tankDrivePID(leftPower, rightPower);
    }

    public void tankDrivePID (double leftGoalPower, double rightGoalPower){
        double deltaTime = (double)(System.currentTimeMillis() - pastTime);

        double leftDistTraveled = getLeftDist() - pastLeftDist;
        double leftVelocity = leftDistTraveled/deltaTime;
        double leftPower = leftDrivePID.update(leftGoalPower, leftVelocity, deltaTime);

        double rightDistTraveled = getRightDist() - pastRightDist;
        double rightVelocity = rightDistTraveled/deltaTime;
        double rightPower = rightDrivePID.update(rightGoalPower, rightVelocity, deltaTime);

        // System.out.println("Left Power: " + leftPower + " ; Right Power: " + rightPower);

        tankDrive(leftPower, rightPower);

        pastTime = System.currentTimeMillis();
        pastLeftDist = getLeftDist();
        pastRightDist = getRightDist();
    }

    public void tankDrive(double leftPower, double rightPower)
    {
        leftMotor1.set(leftPower);
        leftMotor2.set(leftPower);
        rightMotor1.set(rightPower);
        rightMotor2.set(rightPower);
    }

    public void resetEncoders()
    {
        startPosLeft = (leftMotor1.getEncoder().getPosition() + leftMotor2.getEncoder().getPosition())/2;
        startPosRight = (rightMotor1.getEncoder().getPosition() + rightMotor2.getEncoder().getPosition())/2;
    }

    public double getLeftDist()
    {
        //10 cm wheel diameter
        double rawCount = (leftMotor1.getEncoder().getPosition() + leftMotor2.getEncoder().getPosition())/2 - startPosLeft;
        return driveClickToCm * rawCount;
    }

    public double getRightDist()
    {
        double rawCount2 = (rightMotor1.getEncoder().getPosition() + rightMotor2.getEncoder().getPosition())/2 - startPosRight;
        return driveClickToCm * rawCount2;
    }
}