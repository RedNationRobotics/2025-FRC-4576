/*elevatorSubsystem
 * Author : Jacob Ward
 * Contact : Jake52902@gmail.com
 * Date : 02-24-2025
 * Desc: Controls the elevator to raise and lower the arm on 2025 bot
 */

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class trackSubsytem extends SubsystemBase {
    private double currentLoc;
    private double maxSpeed;
    private double desiredLoc;
    private double maxLoc;

    private final Talon elevatorController;
    private final Encoder encoder;
    private final DigitalInput baseLimitSwitch;

    public trackSubsytem(double maxSpeed, double maxDistance, Talon elevatorController, Encoder encoder, DigitalInput baseLimitSwitch){
        this.encoder = encoder;
        currentLoc = encoder.getDistance();
        desiredLoc = currentLoc;
        this.elevatorController = elevatorController;
        this.baseLimitSwitch = baseLimitSwitch;
        this.maxLoc = maxDistance;
    }

    public void setDesiredPosition(double height){
        desiredLoc = height;
    }

    public void drive(double speed){
        if (isAtOrigin()){
            currentLoc = 0;
            stop();
            return;
        }
        if (isTooCloseToBounds() || isAtPosition()){
            stop();
            return;
        }

        //Maybe some sort of spark overflow control?
        elevatorController.set(speed);
    }

    public void driveAbsoluteSpeed(double dir){
        drive(dir * maxSpeed);
    }

    public void goToOrigin(){
        drive(-1*maxSpeed);
    }

    public void stop(){
        drive(0);
    }

    public void driveToDesiredPosition(){
        currentLoc = encoder.getDistance();
        double diff = desiredLoc-currentLoc;
        drive(Math.min(diff, maxSpeed));
    }

    public boolean isAtOrigin(){
        return baseLimitSwitch.get();
    }
    
    public boolean isAtPosition(){
        return (Math.abs(currentLoc - desiredLoc)<.1);
    }

    public boolean isTooCloseToBounds(){
        return (Math.abs(currentLoc - maxLoc)<.1);
    }

}
