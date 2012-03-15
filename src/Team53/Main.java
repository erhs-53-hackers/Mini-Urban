package Team53;

import lejos.nxt.Button;
import lejos.nxt.I2CPort;
import lejos.nxt.LightSensor;
import lejos.nxt.Motor;
import lejos.nxt.MotorPort;
import lejos.nxt.SensorPort;
import lejos.nxt.addon.ColorSensorHT;

public class Main {
    
    public static void main(String[] args) {
        Robot robot = new Robot();

        robot.calibratePilot(3f, 13f);

        robot.setColor(Values.whiteTarget, Values.whiteMax);
        robot.calibratePID(1f, 0.0025f, 0.05f);//L 101, R 116

        //done calibrating starting track
        //robot.checkTachoCount();
        //robot.pilot.forward();
        //while(true) {
        //    System.out.println(Motor.B.getTachoCount());
        //}
        
        robot.hugRight();
        robot.turnRight();
        robot.hugRight();
        robot.turnRight();
        robot.hugRight();
        robot.turnLeft();
        robot.hugLeft();
        //robot.park(2, Direction.Right);
        //robot.getOutOfpark(Direction.Right);
        //robot.hugRight();
        
        //
              
       while(true) {
           System.out.println(robot.RcolorSensor.getRGBComponent(ColorSensorHT.BLACK));
       }
        
    }
}
