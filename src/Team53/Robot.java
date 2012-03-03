package Team53;


import lejos.nxt.LightSensor;
import lejos.nxt.Motor;
import lejos.nxt.SensorPort;
import lejos.nxt.addon.ColorSensorHT;
import lejos.robotics.navigation.DifferentialPilot;
import lejos.util.PIDController;
import lejos.robotics.Color;

public class Robot {
	LightSensor lightSensor;
	ColorSensorHT RcolorSensor;
	ColorSensorHT LcolorSensor;
	PIDController pid;
	DifferentialPilot pilot;
	float speed = 400;
        int parkingSpotLength = 22;
	

	public Robot() {
		RcolorSensor = new ColorSensorHT(SensorPort.S1);
		LcolorSensor = new ColorSensorHT(SensorPort.S2);
		//lightSensor = new LightSensor(SensorPort.S1);
                
                
                LcolorSensor.initBlackLevel();
                RcolorSensor.initBlackLevel();
        try {
            Thread.sleep(5000);
        } catch (InterruptedException ex) {
            //Logger.getLogger(Robot.class.getName()).log(Level.SEVERE, null, ex);
        }
		
	}

	public void calibratePID(float kp, float ki, float kd) {
		pid.setPIDParam(PIDController.PID_KP, kp);
		pid.setPIDParam(PIDController.PID_KI, ki);
		pid.setPIDParam(PIDController.PID_KD, kd);
		
	}
	
	public void setColor(int color) {		
		pid = new PIDController(color);
	}
		
	public void calibratePilot(float wheelDiameter, float trackWidth) {
		pilot = new DifferentialPilot(wheelDiameter, trackWidth, Motor.B, Motor.C);
		pilot.setTravelSpeed(25);
		pilot.setRotateSpeed(30);
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
		System.out.println("R Black: " + RcolorSensor.getRGBComponent(ColorSensorHT.BLACK));
                //System.out.println("R Black: " + RcolorSensor.getRGBComponent(ColorSensorHT.BLACK));
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
	
	public void checkColor(ColorSensorHT sensor) {
            if(sensor.getRGBComponent(ColorSensorHT.WHITE) == 255) {
                speed = 400;
            } else {
                speed = 200;
            }
            System.out.println("sensor:" + sensor.getRGBComponent(ColorSensorHT.BLACK));
		
	}
	
        private String getColor(ColorSensorHT xcolor) 
        {
            int[] xcolorInput = {xcolor.getRGBComponent(Color.RED), xcolor.getRGBComponent(Color.GREEN), xcolor.getRGBComponent(Color.BLUE)};
            int colorAvgValue = (xcolorInput[0] + xcolorInput[1] + xcolorInput[2]) / 3;
            int redVal = (xcolorInput[0] - colorAvgValue);
            int greenVal = (xcolorInput[1] - colorAvgValue);
            int blueVal = (xcolorInput[2] - colorAvgValue);
            if (Math.abs(redVal) < 15 && Math.abs(greenVal) < 15 && Math.abs(blueVal) < 15) {
                if (xcolorInput[0] <= 100 && xcolorInput[1] <= 100 && xcolorInput[2] <= 100) {
                    return "black";
                }
            } else if (xcolorInput[0] > 100 && xcolorInput[1] > 100 && xcolorInput[2] > 100) {
                return "white";
            } else if (redVal > 15 && greenVal <= 0) { //Consideration of blue value unnessesary;
                return "red";
            } else if (redVal > 15 && greenVal > 15) { //Consideration of blue value unnessesary
                return "yellow";
            } else if (redVal <= 0 && greenVal <= 10 && blueVal > 15) {
                return "blue";
            } else if (redVal <= 0 && greenVal > 15) {
                return "green";
            }
            return "???";
    }

	public void hugRight() {
		checkColor(RcolorSensor);
		
		float value = pid.doPID(RcolorSensor.getRGBComponent(ColorSensorHT.BLACK));		
		
		//System.out.println(value);
		
		Motor.B.setSpeed(speed - (speed * (value/128/5)));
		Motor.B.forward();
		Motor.C.setSpeed(speed + (speed * (value/128/5)));
		Motor.C.forward();
                
                
                
		
		//checkForStop(Direction.Left);
		
		
	}
	public void hugLeft() {
            checkColor(LcolorSensor);
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
           /* if (parkingSide == 1)
            {
               if(LcolorSensor.getRGBComponent(ColorSensorHT.BLUE) > 60 & 
                       LcolorSensor.getRGBComponent(ColorSensorHT.BLUE) < 80)
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
                if(LcolorSensor.getRGBComponent(ColorSensorHT.BLUE) > 60 & 
                       LcolorSensor.getRGBComponent(ColorSensorHT.BLUE) < 80)
                { 
                * 
                */
                     pilot.travel(parkingSpotDistance);
                     turnRight();
                     pilot.travel(parkingSpotLength);
                     flag = false;
             /*       
                }
                else
                {
                    hugRight();
                }
            }
            * 
            */
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