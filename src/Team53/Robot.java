package Team53;

import lejos.nxt.LightSensor;
import lejos.nxt.Motor;
import lejos.nxt.SensorPort;
import lejos.nxt.addon.ColorSensorHT;
import lejos.robotics.navigation.DifferentialPilot;
import lejos.util.PIDController;

public class Robot {
	LightSensor lightSensor;
	ColorSensorHT RcolorSensor;
	ColorSensorHT LcolorSensor;
	PIDController pid;
	DifferentialPilot pilot;
	float speed = 400;
	int targetRed, targetWhite, targetYellow, red, yellow, white, currentColor;

	public Robot() {
		RcolorSensor = new ColorSensorHT(SensorPort.S1);
		LcolorSensor = new ColorSensorHT(SensorPort.S2);
		//lightSensor = new LightSensor(SensorPort.S1);
                
                //Thread.sleep(500);
                LcolorSensor.initBlackLevel();
                RcolorSensor.initBlackLevel();
		
	}

	public void calibratePID(float kp, float ki, float kd) {
		pid.setPIDParam(PIDController.PID_KP, kp);
		pid.setPIDParam(PIDController.PID_KI, ki);
		pid.setPIDParam(PIDController.PID_KD, kd);
		
	}
	public void calibrateTargetColors(int white, int red, int yellow) {
		this.targetRed = red;
		this.targetYellow = yellow;
		this.targetWhite = white;
	}
	public void calibrateColors(int white, int red, int yellow) {
		this.white = white;
		this.red = red;
		this.yellow = yellow;
	}
	public void setColor(int color) {
		currentColor = color;
		pid = new PIDController(color);
	}
		
	public void calibratePilot(float wheelDiameter, float trackWidth) {
		pilot = new DifferentialPilot(wheelDiameter, trackWidth, Motor.B, Motor.C);
		pilot.setTravelSpeed(25);
		pilot.setRotateSpeed(30);
	}

	private Team53.Color getColor(ColorSensorHT sensor) {

		Team53.Color c = null;
		int result = 0;
		for (int i = 0; i < 50; i++) {
			result += sensor.getColorID();
		}
		result /= 50;
		System.out.println(result);

		switch (sensor.getColorID()) {
		case 6:
			c = Team53.Color.White;
			break;
		case 3:
			c = Team53.Color.Yellow;
			break;
		case 5:
			c = Team53.Color.Red;
			break;
		case 0:
			c = Team53.Color.Red;
			break;
		case 2:
			c = Team53.Color.Blue;
			break;
		case 8:
			c = Team53.Color.Black;
			break;
		case 1:
			c = Team53.Color.Green;
			break;

		}

		return c;
	}

	public void turnLeft() {
		pilot.rotate(-90);		
	}

	public void turnRight() {
		pilot.rotate(90);
	}
	public void printColors() {
		//System.out.println("Yellow: " + RcolorSensor.getRGBComponent(ColorSensorHT.YELLOW));
		//System.out.println("White: " + RcolorSensor.getRGBComponent(ColorSensorHT.WHITE));
		System.out.println("L Black: " + LcolorSensor.getRGBComponent(ColorSensorHT.BLACK));
                System.out.println("R Black: " + RcolorSensor.getRGBComponent(ColorSensorHT.BLACK));
                //System.out.println("Left Red: " + LcolorSensor.getRGBComponent(ColorSensorHT.RED));
                //System.out.println("Left Green: " + LcolorSensor.getRGBComponent(ColorSensorHT.GREEN));
                //System.out.println("Left Blue: " + LcolorSensor.getRGBNormalized(ColorSensorHT.BLUE));
		

                
           
	}
	void checkForStop(Direction dir) {
		if(dir == Direction.Right) {
			if(RcolorSensor.getRGBComponent(ColorSensorHT.RED) - 255 < 10)
			{
				try {
					Thread.sleep(5000);
				} catch (InterruptedException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}
			}
			System.out.println(RcolorSensor.getRGBComponent(ColorSensorHT.RED));
			
		} else {
			if(LcolorSensor.getRGBComponent(ColorSensorHT.RED) - 255 < 10)
			{
				try {
					Thread.sleep(5000);
				} catch (InterruptedException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}
			}
			System.out.println(LcolorSensor.getRGBComponent(ColorSensorHT.RED));
		}
	}
	
	private void checkColor() {
		System.out.println("Yellow: " + RcolorSensor.getRGBComponent(ColorSensorHT.YELLOW));
		System.out.println("White: " + RcolorSensor.getRGBComponent(ColorSensorHT.WHITE));
	}
	

	public void hugRight() {
		checkColor();
		
		float value = pid.doPID(RcolorSensor.getRGBComponent(ColorSensorHT.BLACK));		
		
		//System.out.println(value);
		
		Motor.B.setSpeed(speed - (speed * (value/128/5)));
		Motor.B.forward();
		Motor.C.setSpeed(speed + (speed * (value/128/5)));
		Motor.C.forward();	
		
		//checkForStop(Direction.Left);
		
		
	}
	public void hugLeft() {
	    float value = pid.doPID(LcolorSensor.getRGBComponent(ColorSensorHT.BLACK));		
		
		//System.out.println(value);
		
		Motor.B.setSpeed(speed + (speed * (value/128/5)));
		Motor.B.forward();
		Motor.C.setSpeed(speed - (speed * (value/128/5)));
		Motor.C.forward();
		
		//checkForStop(Direction.Right);
		
		
	}
        public void park(int parkingSpotDistance, int parkingSide, boolean flag)
        {            
            if (parkingSide == 1)
            {
               if(LcolorSensor.getRGBComponent(ColorSensorHT.BLUE) > 155)
               {
                   pilot.travel(parkingSpotDistance);
                   turnLeft();
                   pilot.travel(parkingSpotLength);
                   flag = true;
               }
               else
               {
                   hugLeft();
               }
            }
            else if (parkingSide == 0)
            {
                if(RcolorSensor.getRGBComponent(ColorSensorHT.BLUE) > 155)
                {
                     pilot.travel(parkingSpotDistance);
                     turnRight();
                     pilot.travel(parkingSpotLength);
                     flag = true;
                }
                else
                {
                    hugRight();
                }
            }
        }
        public void getOutOfpark(int parkingSide)
        {
            pilot.travel(-parkingSpotLength);
            if (parkingSide == 1)
            {
                turnRight();
            }
            else if (parkingSide == 0)
            {
                turnLeft();
            }
        }
}