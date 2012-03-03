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
		robot.calibratePilot(2.4f, 6.8f);
		robot.calibrateTargetColors(140, 0, 84);
		robot.calibrateColors(255, 0, 45);
		robot.setColor(robot.targetWhite);
		robot.calibratePID(1f, 0.005f, 0.2f);
		
		//robot.pilot.setTravelSpeed(20);
		
		//LightSensor l = new LightSensor(SensorPort.S4);
		//l.setFloodlight(true);
		//Motor.A.setSpeed(1);

		while (!Button.ESCAPE.isPressed()) {
                    robot.printColors();
			//System.out.println("Black:"+robot.RcolorSensor.getRGBComponent(ColorSensorHT.BLACK));
			//System.out.println("Yellow:"+robot.RcolorSensor.getRGBComponent(ColorSensorHT.YELLOW));
			//robot.checkForStop(Direction.Right);
			//robot.hugRight();
                    
                    
		
		}

	}

}
