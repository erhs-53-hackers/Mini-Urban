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
		
		robot.setColor(103);
		robot.calibratePID(1f, 0.005f, 0.2f);//L 101, R 116
		
		//robot.pilot.setTravelSpeed(20);
		
		//LightSensor l = new LightSensor(SensorPort.S4);
		//l.setFloodlight(true);
		//Motor.A.setSpeed(1);
                //robot.pilot.setTravelSpeed(100);
                //robot.pilot.travel(50);

		
                   // robot.printColors();
			//System.out.println("Black:"+robot.RcolorSensor.getRGBComponent(ColorSensorHT.BLACK));
			//System.out.println("Yellow:"+robot.RcolorSensor.getRGBComponent(ColorSensorHT.YELLOW));
			//robot.checkForStop(Direction.Right);


                    //robot.hugRight();
                    //robot.turnRight();
                    //robot.hugRight();
                    //robot.turnRight();
                    //robot.hugRight();
                    //robot.turnLeft();
                    //robot.hugLeft();
                    robot.park(3, 0, false);
                    //robot.checkColor(robot.RcolorSensor);
                    
			//robot.hugRight
                    /* 
                    while(flag || !Button.ESCAPE.isPressed())
                    {
                       robot.park(20, 0, flag);
                    }
                    * */

                    
                    //System.out.println(robot.RcolorSensor.getRGBComponent(ColorSensorHT.BLUE));

                    
		
		

	}

}
