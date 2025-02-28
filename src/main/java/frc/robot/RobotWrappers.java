package frc.robot;

public class RobotWrappers {
    /*
     * The reasoning behind this class and the wrappers within is wpi/rev/whoever can't make
     * up their minds on using existing APIs and interfaces properly, and everyone has their
     * own special snowflake solution. Because of this, to properly decouple ourselves from
     * the implementations of each of these things (which is useful for modularity, code re-use
     * and maintanability) we need to have interfaces (or functional interfaces in this case)
     * that can wrap various interfaces from other libraries easily. Since this is as much
     * of a mess as it is, we should investigate setting up some initialization functionality
     * that can extract pieces into interfaces like this easily, into easy packages that
     * can be passed to wherever they need to go. This will make the code more readable, 
     * and less dumb to use (using this class as it is is going to be a bit ugly because
     * of the million wrappers you have to pass in, but it's moving towards a solution
     * that will hopefully make everything cleaner) - Bryan 3/2/24
     */

    @FunctionalInterface
    public interface EncoderCountWrapper {
        int getEncoderCount();
    }

    public interface EncoderWrapper { //possibly combine with the one above
        int getEncoderCount();
        void resetEncoder();
    }

    @FunctionalInterface
    public interface MotorWrapper {
        void set(double speed);
    }

    @FunctionalInterface
    public interface InitCallback {
        void init();
    }
}