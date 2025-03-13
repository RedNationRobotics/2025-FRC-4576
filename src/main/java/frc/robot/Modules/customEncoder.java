package frc.robot.Modules;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class customEncoder extends SubsystemBase  {
    public double rotations;
    double negativeRotations;
    double negative;
    public double prevRotation;
    DutyCycleEncoder encoder;

    public customEncoder(DutyCycleEncoder encoder){
        this.encoder = encoder;
        rotations = 0;
        negative = 0;
        prevRotation = getRelVal();
    }

    @Override
    public void periodic() {
        // TODO Auto-generated method stub
        super.periodic();
        updateDistance();
    }

    public void updateDistance(){
        int roughDist = (int)(getRelVal() * 10);
        int roughPrev = (int)(prevRotation * 10);
        if ((roughDist>=8) && (roughPrev<=2) ) {
            rotations-=1;
        }
        if ((roughPrev>=8) && (roughDist<=2) ) {
            rotations+=1;
        }
        prevRotation = getRelVal();
    }

    public double getRot(){
        return rotations + getRelVal();
    }

    public void zero(){
        negativeRotations = ((int) rotations);
        rotations = 0;
        negative = encoder.get();
        prevRotation = getRelVal();
    }

    public double getRelVal(){
        double value = encoder.get();
        return value - ((int)value);
    }

    public double getValue(){
        return (rotations - negativeRotations -negative + getRelVal());
    }
}
