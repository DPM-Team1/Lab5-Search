package ca.mcgill.ecse211.Lab5;

public interface UltrasonicController {

  public boolean processUSData(int distance);

  public int readUSDistance();
}
