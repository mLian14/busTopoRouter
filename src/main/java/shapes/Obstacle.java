package shapes;

import java.util.*;


/**
 * @auther lianmeng
 * @create 25.03.23
 */
public class Obstacle {

    private String name;

    private int minX, maxX, minY, maxY;

    private PseudoBase lowerLeft, lowerRight, upperLeft, upperRight;
    private ArrayList<PseudoBase> baseArray;

    private ArrayList<Obstacle> tLObstacles, tRObstacles, bLObstacles, bRObstacles;





    public Obstacle(String name, int minX, int maxX, int minY, int maxY) {
        this.name = name;
        this.minX = minX;
        this.maxX = maxX;
        this.minY = minY;
        this.maxY = maxY;

        this.lowerLeft = new PseudoBase(minX, minY);
        this.lowerLeft.setType(BaseType.lowerLeft);
        this.lowerRight = new PseudoBase(maxX, minY);
        this.lowerRight.setType(BaseType.lowerRight);
        this.upperLeft = new PseudoBase(minX, maxY);
        this.upperLeft.setType(BaseType.upperLeft);
        this.upperRight = new PseudoBase(maxX, maxY);
        this.upperRight.setType(BaseType.upperRight);
        this.baseArray = new ArrayList<>(Arrays.asList(this.lowerLeft, this.lowerRight, this.upperLeft, this.upperRight));

        this.tLObstacles = new ArrayList<>();
        this.tRObstacles = new ArrayList<>();
        this.bLObstacles = new ArrayList<>();
        this.bRObstacles = new ArrayList<>();

    }

    public ArrayList<PseudoBase> getBaseArray() {
        return baseArray;
    }

    public PseudoBase getLowerLeft() {
        return lowerLeft;
    }

    public PseudoBase getLowerRight() {
        return lowerRight;
    }

    public PseudoBase getUpperLeft() {
        return upperLeft;
    }

    public PseudoBase getUpperRight() {
        return upperRight;
    }

    public String getName() {
        return name;
    }

    public void setName(String name) {
        this.name = name;
    }

    public int getMinX() {
        return minX;
    }

    public void setMinX(int minX) {
        this.minX = minX;
    }

    public int getMaxX() {
        return maxX;
    }

    public void setMaxX(int maxX) {
        this.maxX = maxX;
    }

    public int getMinY() {
        return minY;
    }

    public void setMinY(int minY) {
        this.minY = minY;
    }

    public int getMaxY() {
        return maxY;
    }

    public void setMaxY(int maxY) {
        this.maxY = maxY;
    }

    public ArrayList<Obstacle> gettLObstacles() {
        return tLObstacles;
    }

    public void addTotLObstacles(Obstacle o) {
        this.tLObstacles.add(o);
    }

    public ArrayList<Obstacle> gettRObstacles() {
        return tRObstacles;
    }

    public void addTotRObstacles(Obstacle o) {
        this.tRObstacles.add(o);
    }

    public ArrayList<Obstacle> getbLObstacles() {
        return bLObstacles;
    }

    public void addTobLObstacles(Obstacle o) {
        this.bLObstacles.add(o);
    }

    public ArrayList<Obstacle> getbRObstacles() {
        return bRObstacles;
    }

    public void addTobRObstacles(Obstacle o) {
        this.bRObstacles.add(o);
    }

    public String convertArrayToString(ArrayList<Obstacle> obstacles) {

        StringBuilder arrayAsString = new StringBuilder("||");
        for (Obstacle o : obstacles) {
            arrayAsString.append(o.getName() + "; ");
        }
        arrayAsString.append("||");
        arrayAsString.append("\n");
        return arrayAsString.toString();
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (o == null || getClass() != o.getClass()) return false;
        Obstacle obstacle = (Obstacle) o;
        return minX == obstacle.minX && maxX == obstacle.maxX && minY == obstacle.minY && maxY == obstacle.maxY;
    }

    public boolean onLeft(Obstacle other_o){
        return other_o.getMinX() < this.getMinX();
    }

    public boolean onRight(Obstacle other_o){
        return other_o.getMaxX() > this.getMaxX();
    }

    public boolean onTop(Obstacle other_o){
        return other_o.getMaxY() > this.getMaxY();
    }

    public boolean onBottom(Obstacle other_o){
        return other_o.getMinY() < this.getMinY();
    }

    public boolean down_AreaOverlap(Obstacle other_o){
        return other_o.minY + other_o.minX < this.maxY + this.maxX || other_o.maxY + other_o.maxX > this.minY + this.minX;
    }

    public boolean up_AreaOverlap(Obstacle other_o){
        return other_o.minY - other_o.maxX < this.maxY - this.maxX || other_o.maxY - other_o.minX > this.minY - this.maxX;
    }

    public boolean topL_bottomR_oo(Obstacle other_o){
        return this.minX <= other_o.maxX && this.maxY >= other_o.maxY;
    }

    public boolean bottomR_topL_oo(Obstacle other_o){
        return this.maxX >= other_o.minX && this.minY <= other_o.maxY;
    }

    public boolean topR_bottomL_oo(Obstacle other_o){
        return this.maxX >= other_o.minX && this.maxY >= other_o.minY;
    }

    public boolean bottomL_topR_oo(Obstacle other_o){
        return this.minX <= other_o.maxX && this.minY <= other_o.maxY;
    }


    @Override
    public int hashCode() {
        return Objects.hash(minX, maxX, minY, maxY);
    }

    @Override
    public String toString() {
        return "Obstacle{" +
                "name='" + name + '\'' +
                ", minX=" + minX +
                ", maxX=" + maxX +
                ", minY=" + minY +
                ", maxY=" + maxY + "\n" +
                ", OtL=" + convertArrayToString(tLObstacles) + "\n" +
                ", OtR=" + convertArrayToString(tRObstacles) + "\n" +
                '}';
    }
}
