package frc.robot.Commands;

import com.revrobotics.spark.SparkMax;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;



public class Motor extends Command {

    private final DigitalInput LimitSwitchOutEnd, LimitSwitchOrigin; 
    private final TalonSRX motor;


    public Motor( DigitalInput LimitSwitchOutEnd, DigitalInput LimitSwitchOrigin, TalonSRX motor){

        this.LimitSwitchOutEnd = LimitSwitchOutEnd;
        this.LimitSwitchOrigin = LimitSwitchOrigin;
        this.motor = motor;

        
    }

    public void execute() {
        if(LimitSwitchOutEnd.get())end(
            false);
        motor.set(TalonSRXControlMode.PercentOutput, .1);
    }

        

    }

