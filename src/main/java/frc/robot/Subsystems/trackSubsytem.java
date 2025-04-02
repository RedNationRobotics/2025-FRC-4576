/*elevatorSubsystem
 * Author : Jacob Ward
 * Contact : Jake52902@gmail.com
 * Date : 02-24-2025
 * Desc: Controls the elevator to raise and lower the arm on 2025 bot
 */

package frc.robot.Subsystems;

import com.revrobotics.spark.SparkBase;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Modules.customEncoder;

public class trackSubsytem extends SubsystemBase {
    private double currentLoc;
    private double maxSpeed;
    private double desiredLoc;
    private double maxLoc;

    private final SparkBase elevatorController;
    public final customEncoder encoder;
    private final DigitalInput baseLimitSwitch;

    private static final double margin = .01;
    private final double holdSpeed;

    public trackSubsytem(double maxSpeed, double maxDistance, SparkBase elevatorController, customEncoder encoder, DigitalInput baseLimitSwitch, double holdSpeed){
        this.maxSpeed = maxSpeed;
        this.encoder = encoder;
        currentLoc = encoder.getValue();
        desiredLoc = .1;
        this.elevatorController = elevatorController;
        this.baseLimitSwitch = baseLimitSwitch;
        this.maxLoc = maxDistance;
        this.holdSpeed = holdSpeed;
    }

    public void setDesiredPosition(double height){
        desiredLoc = height;
    }

    public void drive(double speed, double target){
       if (isAtOrigin() && speed<0.0){
            currentLoc = 0;
            stop();
            return;
        }
        if (wouldBeTooCloseToBounds((Math.signum(speed)*.01)) || isAtPosition(target)){
            stop();
            return;
        }
        
        double distToTarget = Math.abs(curDistanceToPoint(target));
        if (distToTarget<.5) speed *= .5;
        speed = (distToTarget < .2 && distToTarget > 0) ? .2 * Math.signum(speed) : speed;

        double modifier = 1;
        if (currentLoc < .5 && speed<0){
            modifier = .2;
        }
        elevatorController.set(MathUtil.clamp(((speed * maxSpeed)+holdSpeed)* modifier,-.5,1.0));
    }

    public void driveAbsoluteSpeed(double dir){
        double modifier = 1;
        if (currentLoc < .5 && dir<0){
            modifier = .2;
        }
        elevatorController.set(dir * maxSpeed * modifier);
    }

    public void stop(){
        elevatorController.set(holdSpeed);
    }

    public void driveToDesiredPosition(){
        double diff = desiredLoc-currentLoc;
        drive(MathUtil.clamp(diff, -.5, 1.0), desiredLoc);
    }

    public void driveToPosition(double target){
        double diff = target-currentLoc;
        drive(MathUtil.clamp(diff, -.5, 1.0), target);
    }

    public boolean isAtOrigin(){
        return !baseLimitSwitch.get();
    }
    
    public boolean isAtPresetPosition(){
        return isAtPosition(desiredLoc);
    }

    public boolean isAtPosition(double target){
        return (Math.abs(currentLoc - target)<.09);
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
        super.periodic();
        currentLoc = encoder.getValue();
        if (baseLimitSwitch.get() == false){
            encoder.zero();
            stop();
        }
        
    }

}
