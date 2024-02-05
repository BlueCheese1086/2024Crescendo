package Util;

public class CheesyUnits {
    
    public double radiansToCheesians(double rad) {
        return rad/Math.PI/2.0*1086.0;
    }

    public double cheesiansToRadians(double cheese) {
        return cheese/1086.0*2.0*Math.PI;
    }

    /**
     * Leaving it to cheese cuz it rhymes
     */
    public double cheeseToDegrees(double cheese) {
        return cheese/1086.0*360.0;
    }

    /**
     * Leaving it to cheese cuz it rhymes
     */
    public double degreesToCheese(double degrees) {
        return degrees/360.0*1086.0;
    }


}
