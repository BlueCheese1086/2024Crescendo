package Util.Interfaces;

public interface PowerManaged {

    /**
     * @return Returns the total current being drawn by the subsystem
     */
    public double getTotalCurrent();

    /**
     * @return Returns the total current limit of the subsystsem
     */
    public  double getCurrentLimit();

    /**
     * Contains instructions on what to do in case extra current draw is detected
     */
    default public void overCurrentDetection() {};

    /**
     * Sets the current limit of the overall subsystem
     * @param a The current in amps
     */
    public void setCurrentLimit(int a);
}
