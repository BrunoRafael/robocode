package sevenrobot;
import robocode.*;
import java.awt.Color;
import java.awt.geom.*;
import java.util.*;
import java.util.concurrent.TimeUnit;
import robocode.util.Utils;


public class Seven extends AdvancedRobot
{
	Enemy target;
	final double PI = Math.PI;
	double firePower;
	double midpointstrength, countBullets = 0;
	int midpointcount = 0;
	int movementDirection = 1;
	
	public void run() {
		target = new Enemy();
		target.distance = 100000;
		setBodyColor(Color.yellow);
		setGunColor(Color.green);
		setRadarColor(Color.black);
		setScanColor(Color.red);
		setBulletColor(Color.yellow);

		setAdjustGunForRobotTurn(true);
		setAdjustRadarForGunTurn(true);
		turnRadarRightRadians(2*PI);
		while(true) {
			antiGravMove();
			setTurnRadarLeftRadians(2*PI);
			rotateGun();
			execute();
		}
	}
	
	// O robô se move segundo a técnica de anti-gravidade implementada juntamente com um movimento circular do canhão
	private void antiGravMove() {
		double force = (-1000/Math.pow(getDistanceToEnemy(getX(),getY(),target.x, target.y),2))* 2;
		double ang = normaliseBearing(Math.PI/2 - Math.atan2(getY() - target.y, getX() - target.x)); 
		double xforce = Math.sin(ang) * force;
		double yforce = Math.cos(ang) * force;
	    
		midpointcount++;
		if (midpointcount > 5) {
			midpointcount = 0;
			midpointstrength = (Math.random() * 2000) - 1000;
		}
		
		GravityPoint p = new GravityPoint(getBattleFieldWidth()/2, getBattleFieldHeight()/2, midpointstrength);
		force = p.power/Math.pow(getDistanceToEnemy(getX(),getY(),p.x,p.y),1.5);
	    ang = normaliseBearing(Math.PI/2 - Math.atan2(getY() - p.y, getX() - p.x)); 
	    xforce += Math.sin(ang) * force;
	    yforce += Math.cos(ang) * force;

	    xforce += 5000/Math.pow(getDistanceToEnemy(getX(), getY(), getBattleFieldWidth(), getY()), 3);
	    xforce -= 5000/Math.pow(getDistanceToEnemy(getX(), getY(), 0, getY()), 3);
	    yforce += 5000/Math.pow(getDistanceToEnemy(getX(), getY(), getX(), getBattleFieldHeight()), 3);
	    yforce -= 5000/Math.pow(getDistanceToEnemy(getX(), getY(), getX(), 0), 3);
	    
	    double angle = Math.toDegrees(absbearing(getX(),getY(),getX()-xforce,getY()-yforce));
	    double r = turnTo(angle);
		setAhead(20 * r);
		if(angle <= 0 && r <= 0) {
			movementDirection = -movementDirection;
		}
	}

	private int turnTo(double angle) {
	    double ang;
    	int dir;
	    ang = getHeading() - angle;
	    if (ang > 120) {
	        ang -= 180;
	        dir = -1;
	    }
	    else if (ang < -120) {
	        ang += 180;
	        dir = -1;
	    }
	    else {
	        dir = 1;
	    }

	    return dir;
	}

	private void rotateGun() {
		double firePower = 400/target.distance;
		if (firePower > 3) {
			firePower = 3;
		}

		long time = getTime() + (int)Math.round((getDistanceToEnemy(getX(),getY(),target.x,target.y)/(20-(3*firePower))));
		Point2D.Double p = target.guessPosition(time, target.speed);
		double gunOffset = getGunHeadingRadians() - (Math.PI/2 - Math.atan2(p.y - getY(), p.x - getX()));
		setTurnGunLeftRadians(normaliseBearing(gunOffset));
		fire(firePower);
				
	}
	
	private double normaliseBearing(double ang) {
		if (ang > PI)
			ang -= 2*PI;
		if (ang < -PI)
			ang += 2*PI;
		return ang;
	}
	
	private double getDistanceToEnemy( double x1,double y1, double x2,double y2 )
	{
		double x = x2-x1;
		double y = y2-y1;
		return Math.sqrt( Math.pow(x, 2) + Math.pow(y, 2) );
	}
	
	private double absbearing( double x1,double y1, double x2,double y2 )
	{
		double h = getDistanceToEnemy( x1,y1, x2,y2 );
		double x = x2-x1;
		double y = y2-y1;
		if( x > 0 && y > 0 )
		{
			return Math.asin( x / h );
		}
		if( x > 0 && y < 0 )
		{
			return Math.PI - Math.asin( x / h );
		}
		if( x < 0 && y < 0 )
		{
			return Math.PI + Math.asin( -x / h );
		}
		if( x < 0 && y > 0 )
		{
			return 2.0*Math.PI - Math.asin( -x / h );
		}
		return 0;
	}

	public void onScannedRobot(ScannedRobotEvent e) {
		double bearingInRad = (getHeadingRadians()+e.getBearingRadians())%(2*PI);
		target.name = e.getName();
		target.x = getX()+Math.sin(bearingInRad)*e.getDistance();
		target.y = getY()+Math.cos(bearingInRad)*e.getDistance();
		target.bearing = e.getBearingRadians();
		target.heading = e.getHeadingRadians();
		target.oldTime = getTime();
		target.speed = e.getVelocity();
		target.distance = e.getDistance();	

		setTurnRight(e.getBearing()+120 - 30 * movementDirection);
		setAhead((e.getDistance()/4+25) * movementDirection);
	}

	public void onHitWall(HitWallEvent e) {
		movementDirection = -movementDirection;
		setAhead(20* movementDirection);
	}	
}

class Enemy {
	String name;
	public double bearing,heading,speed,x,y,distance;
	public long oldTime;
	public Point2D.Double guessPosition(long when, double oldSpeed) {
		double diff = when - oldTime;
		double newY = y + Math.cos(heading) * Math.min(oldSpeed, speed) * diff;
		double newX = x + Math.sin(heading) * Math.min(oldSpeed, speed) * diff;
		
		return new Point2D.Double(newX, newY);
	}
}

class GravityPoint {
    public double x,y,power;
    public GravityPoint(double X,double Y,double Power) {
        x = X;
        y = Y;
        power = Power;
    }
}
