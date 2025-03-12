package frc.robot.Modules;

import com.revrobotics.spark.SparkFlex;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class customEncoder extends SubsystemBase  {
    double rotations;
    double negative;
    double prevRotation;
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
        prevRotation = encoder.get();
    }

    public double getRot(){
        return rotations + getRelVal();
    }

    public void zero(){
        rotations = 0;
        negative = encoder.get();
    }

    public double getRelVal(){
        double value = encoder.get() + negative;
        return value - (double)((int)value);
    }

    public double getValue(){
        return rotations + encoder.get();
    }
}
