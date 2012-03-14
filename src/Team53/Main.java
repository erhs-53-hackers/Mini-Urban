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

        robot.setColor(Values.whiteTarget);
        robot.calibratePID(1f, 0.005f, 0.2f);//L 101, R 116

        //done calibrating starting track
        robot.checkTachoCount();
        
        robot.hugRight();
        robot.turnRight();
        robot.hugRight();
        robot.turnRight();
        robot.hugRight();
        robot.turnLeft();
        robot.hugLeft();
        
    }
}
