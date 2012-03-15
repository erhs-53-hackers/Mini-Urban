package Team53;


import lejos.nxt.Button;
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
    final int parkingSpotLength = 22;
    final int parkingSpotDistance = 17;
    int max = 1;
    float speed = 600;
    int lastValue = 0, target;
    float P, I, D;

    public Robot() {
        RcolorSensor = new ColorSensorHT(SensorPort.S1);
        LcolorSensor = new ColorSensorHT(SensorPort.S2);
        //lightSensor = new LightSensor(SensorPort.S1);
        System.out.println("Press left to calibrate");
        while (!Button.LEFT.isPressed()) {
        }
        System.out.println("Calibrating...");
        LcolorSensor.initBlackLevel();
        RcolorSensor.initBlackLevel();
        System.out.println("Press enter to start");
        while (!Button.ENTER.isPressed()) {
        }
    }

    public void calibratePID(float kp, float ki, float kd) {
        P = kp;
        I = ki;
        D = kd;

        pid.setPIDParam(PIDController.PID_KP, kp);
        pid.setPIDParam(PIDController.PID_KI, ki);
        pid.setPIDParam(PIDController.PID_KD, kd);

    }

    private void resetPID(int color, int max) {
        target = color;
        this.max = max;
        pid = new PIDController(color);
        pid.setPIDParam(PIDController.PID_KP, P);
        pid.setPIDParam(PIDController.PID_KI, I);
        pid.setPIDParam(PIDController.PID_KD, D);
    }

    public void setColor(int color, int max) {
        target = color;
        this.max = max;
        pid = new PIDController(color);
    }

    public void calibratePilot(float wheelDiameter, float trackWidth) {
        pilot = new DifferentialPilot(wheelDiameter, trackWidth, Motor.C, Motor.B);
        pilot.setTravelSpeed(25);
        pilot.setRotateSpeed(10);

    }

    public void turnLeft() {
        System.out.println("Turning...");
        pilot.travel(5);
        pilot.setTravelSpeed(2);
        findLine(Direction.Left);

        pilot.setTravelSpeed(25);
    }

    public void turnRight() {
        System.out.println("Turning...");
        pilot.travel(5);
        pilot.setTravelSpeed(2);
        findLine(Direction.Right);

        pilot.setTravelSpeed(25);
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

    boolean checkForStop(ColorSensorHT sensor) {

        if ("red".equals(getColor(sensor))) {
            return true;
        }
        return false;

    }

    public void checkTachoCount() {
        System.out.println(Motor.B.getTachoCount());
    }

    public void checkColor(ColorSensorHT sensor) {
        if ("white".equals(getColor(sensor))) {
            speed = 400;
            if(target != Values.whiteTarget) {
                resetPID(Values.whiteTarget, Values.whiteMax);
                
            }
        } else if ("yellow".equals(getColor(sensor))) {
            speed = 392;
            if(target != Values.yellowTarget) {
                resetPID(Values.yellowTarget, Values.yellowMax);
            }
        } else if ("blue".equals(getColor(sensor))) {
            speed = 400;
            if(target != Values.blueTarget) {
                resetPID(Values.blueTarget, Values.blueMax);
            }
        }
        //System.out.println("sensor:" + sensor.getRGBComponent(ColorSensorHT.BLACK));

    }

    public void goStrait() {
        pilot.travel(Values.streetWidth);
    }

    public void findLine(Direction dir) {
        if (dir == Direction.Left) {
            Motor.B.setSpeed(500);
            Motor.C.setSpeed(100);
            Motor.B.forward();
            Motor.C.forward();
            while (!"white".equals(getColor(LcolorSensor))) {
                System.out.println(getColor(LcolorSensor));
            }
            pilot.stop();
        } else {
            Motor.B.setSpeed(100);
            Motor.C.setSpeed(500);
            Motor.B.forward();
            Motor.C.forward();

            while ("black".equals(getColor(RcolorSensor))) {
                System.out.println(getColor(RcolorSensor));
            }
            pilot.stop();
        }
    }

    private String getColor(ColorSensorHT xcolor) {
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

    public void adjustPath(Direction dir) {
        if (dir == Direction.Right) {
            checkColor(RcolorSensor);
            float value = pid.doPID(RcolorSensor.getRGBComponent(ColorSensorHT.BLACK));

            Motor.B.setSpeed(speed - (speed * (value / max / 3)));
            Motor.B.forward();
            Motor.C.setSpeed(speed + (speed * (value / max / 3)));
            Motor.C.forward();
        } else {
            checkColor(LcolorSensor);

            float value = pid.doPID(LcolorSensor.getRGBComponent(ColorSensorHT.BLACK));

            Motor.B.setSpeed(speed + (speed * (value / max / 3)));
            Motor.B.forward();
            Motor.C.setSpeed(speed - (speed * (value / max / 3)));
            Motor.C.forward();
        }

    }

    public void hugRight() {
        resetPID(target, max);
        while (!checkForStop(LcolorSensor)) {


            adjustPath(Direction.Right);

            System.out.println("Following...");


        }
        pilot.stop();



        try {
            Thread.sleep(1000);

            //checkForStop(Direction.Right);
        } catch (InterruptedException ex) {
            //Logger.getLogger(Robot.class.getName()).log(Level.SEVERE, null, ex);
        }

        //checkForStop(Direction.Left);


    }

    public void hugLeft() {
        resetPID(target, max);
        while (!checkForStop(RcolorSensor)) {

            adjustPath(Direction.Left);

            System.out.println("Following...");



        }
        pilot.stop();
        try {
            Thread.sleep(1000);

            //checkForStop(Direction.Right);
        } catch (InterruptedException ex) {
            //Logger.getLogger(Robot.class.getName()).log(Level.SEVERE, null, ex);
        }


    }

    public void park(int spotNum, Direction dir) {
        ColorSensorHT primary = LcolorSensor, secondary = RcolorSensor;
        if (dir == Direction.Right) {
            primary = RcolorSensor;
            secondary = LcolorSensor;
        }
        resetPID(Values.whiteTarget, Values.whiteMax);
        while (true) {
            if ("blue".equals(getColor(primary))) {
                break;
            }
            System.out.println("Following not blue...");
            adjustPath(dir);
        }
        resetPID(Values.blueTarget, Values.blueMax);
        Motor.B.resetTachoCount();
        while (Motor.B.getTachoCount() < 680 * (spotNum - 1)) {
            System.out.println("Following blue...");
            //System.out.println(Motor.B.getTachoCount() + " needs " + 680 * (spotNum - 1));
            adjustPath(dir);
        }

        pilot.setTravelSpeed(10);
        pilot.setRotateSpeed(30);
        pilot.travel(13);
        if(dir == Direction.Right) {
            pilot.rotate(-40);
        } else {
            pilot.rotate(40);
        }
        
        findLine(dir);
        resetPID(Values.whiteTarget, Values.whiteMax);
        while (!"white".equals(getColor(secondary))) {
            adjustPath(dir);
            System.out.println("Parking...");
        }

        System.out.println("Parked!");
        try {
            pilot.stop();
            Thread.sleep(5000);

            //checkForStop(Direction.Right);
        } catch (InterruptedException ex) {
            //Logger.getLogger(Robot.class.getName()).log(Level.SEVERE, null, ex);
        }

    }

    //slow 6m /min
    //fast 10 m /min
    public void getOutOfpark(Direction dir) {
        ColorSensorHT primary = LcolorSensor, secondary = RcolorSensor;
        if (dir == Direction.Right) {
            primary = RcolorSensor;
            secondary = LcolorSensor;
        }
        pilot.travel(-parkingSpotLength);
        if (dir == Direction.Right) {
            pilot.rotate(90);
        } else if (dir == Direction.Left) {
            pilot.rotate(-90);
        }
        resetPID(Values.blueTarget, Values.blueMax);
        while (true) {
            System.out.println("Color " + getColor(primary));
            if("white".equals(getColor(primary)) || "yellow".equals(getColor(primary))) {
                break;
            }
            adjustPath(dir);
            
            
        }
        System.out.println("I hate Lisp");
        
    }
}