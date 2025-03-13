/*elevatorSubsystem
 * Author : Jacob Ward
 * Contact : Jake52902@gmail.com
 * Date : 02-24-2025
 * Desc: Controls the elevator to raise and lower the arm on 2025 bot
 */

package frc.robot.Subsystems;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkFlex;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Modules.customEncoder;

public class trackSubsytem extends SubsystemBase {
    private double currentLoc;
    private double maxSpeed;
    private double desiredLoc;
    private double maxLoc;

    private final SparkBase elevatorController;
    private final customEncoder encoder;
    private final DigitalInput baseLimitSwitch;

    private static final double margin = .01;
    private final double holdSpeed;
    private double speed;
    private double prevWorkingSpeed;

    public trackSubsytem(double maxSpeed, double maxDistance, SparkBase elevatorController, customEncoder encoder, DigitalInput baseLimitSwitch, double holdSpeed){
        this.maxSpeed = maxSpeed;
        this.encoder = encoder;
        currentLoc = encoder.getValue();
        desiredLoc = .1;
        this.elevatorController = elevatorController;
        this.baseLimitSwitch = baseLimitSwitch;
        this.maxLoc = maxDistance;
        this.holdSpeed = holdSpeed;
        speed = 0;
        prevWorkingSpeed = 1;
    }

    public void setDesiredPosition(double height){
        desiredLoc = height;
    }

    public void drive(double speed){
       if (isAtOrigin() && speed<0.0){
            //System.out.println("AT ORIGIN");
            currentLoc = 0;
            stop();
            return;
        }
        if (wouldBeTooCloseToBounds((Math.signum(speed)*.01)) || isAtPosition()){
            //System.out.println("TOO CLOSE TO BOUNDS");
            stop();
            return;
        }
        if (speed != 0){
            prevWorkingSpeed = speed;
        }
        
        double distToTarget = Math.abs(curDistanceToPoint(desiredLoc));
        if (distToTarget<.5) speed *= (distToTarget *2);
        speed = (distToTarget < .2 && distToTarget > 0) ? .2 * Math.signum(speed):speed; 
        this.speed = speed;
        //System.out.println("SPEED IS " + speed);
        elevatorController.set((speed*maxSpeed)+holdSpeed);
    }

    public void driveAbsoluteSpeed(double dir){
        elevatorController.set(dir * maxSpeed);
    }

    public void stop(){
        elevatorController.set(holdSpeed);
    }

    public void driveToDesiredPosition(){
        double diff = desiredLoc-currentLoc;
        drive(Math.signum(diff)*Math.min(Math.abs(diff), 1.0));
    }

    public boolean isAtOrigin(){
        return !baseLimitSwitch.get();
    }
    
    public boolean isAtPosition(){
        return (Math.abs(currentLoc - desiredLoc)<.09);
    }

    public boolean isTooCloseToBounds(){
        return Math.abs(currentLoc - maxLoc)<.1;
    }
    
    public boolean wouldBeTooCloseToBounds(double value){
        return (Math.abs(value - maxLoc)<.1) || value > maxLoc;
    }

    public double curDistanceToPoint(double p){
        return p - currentLoc;
    }
    
    public double pointLimitSpeed(double speed, double point){
        return speed * (curDistanceToPoint(point)/(margin*1.1));
    }

    @Override
    public void periodic() {
        // TODO Auto-generated method stub
        super.periodic();
        currentLoc = encoder.getValue();
        if (baseLimitSwitch.get() == false){
            encoder.zero();
            stop();
        }
        
    }

}
