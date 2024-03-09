package Util.Interfaces;

public interface PowerManaged {
    public double getTotalCurrent();
    public double getCurrentLimit();
    public void overCurrentDetection();
    public void setCurrentLimit(int a);
}
