package shapes;

import java.util.*;


/**
 * @auther lianmeng
 * @create 25.03.23
 */
public class Obstacle {

    private String name;

    private int minX, maxX, minY, maxY;

    private double delta_x, delta_y, gradient;

    private PseudoBase lowerLeft, lowerRight, upperLeft, upperRight;
    private ArrayList<PseudoBase> baseArray;

    private ArrayList<Obstacle> tLObstacles, tRObstacles, bLObstacles, bRObstacles;

    private ArrayList<Obstacle> dLObstacles, dRObstacles, dTObstacles, dBObstacles;

    /*
    map_oo_distMin
    0: ll->ll
    1: ll->ul
    2: ll->ur
    3: ll->lr

    4: ul->ll
    5: ul->ul
    6: ul->ur
    7: ul->lr

    8: ur->ll
    9: ur->ul
    10: ur->ur
    11: ur->lr

    12: lr->ll
    13: lr->ul
    14: lr->ur
    15: lr->lr
     */
    private Map<Obstacle, int[]> oo_distMin;

    /*
    map_oo_distXY
     */
    private Map<Obstacle, int[]> oo_distXY;

    /*
    0: ll
    1: ul
    2: ur
    3: lr
     */
    private Map<PseudoBase, Map<PseudoBase, Integer>> oBase_distMin;//first base is the master/slaves
    private Map<PseudoBase, Map<PseudoBase, Integer>> oBase_distXY;


    public Obstacle(String name, int minX, int maxX, int minY, int maxY) {
        this.name = name;
        this.minX = minX;
        this.maxX = maxX;
        this.minY = minY;
        this.maxY = maxY;

        this.delta_x = this.maxX - this.minX;
        this.delta_y = this.maxY - this.minY;
        this.gradient = this.delta_y / this.delta_x;

        this.baseArray = new ArrayList<>();

        this.lowerLeft = new PseudoBase(minX, minY);
        this.lowerLeft.setType(BaseType.lowerLeft);
        this.lowerLeft.setName(this.name + ".ll");
        this.baseArray.add(this.lowerLeft);


        this.upperLeft = new PseudoBase(minX, maxY);
        this.upperLeft.setType(BaseType.upperLeft);
        this.upperLeft.setName(this.name + ".ul");
        this.baseArray.add(this.upperLeft);

        this.upperRight = new PseudoBase(maxX, maxY);
        this.upperRight.setType(BaseType.upperRight);
        this.upperRight.setName(this.name + ".ur");
        this.baseArray.add(this.upperRight);

        this.lowerRight = new PseudoBase(maxX, minY);
        this.lowerRight.setType(BaseType.lowerRight);
        this.lowerRight.setName(this.name + ".lr");
        this.baseArray.add(this.lowerRight);


        this.tLObstacles = new ArrayList<>();
        this.tLObstacles.add(this);
        this.tRObstacles = new ArrayList<>();
        this.tRObstacles.add(this);
        this.bLObstacles = new ArrayList<>();
        this.bLObstacles.add(this);
        this.bRObstacles = new ArrayList<>();
        this.bRObstacles.add(this);

        this.dLObstacles = new ArrayList<>();
        this.dLObstacles.add(this);
        this.dRObstacles = new ArrayList<>();
        this.dRObstacles.add(this);
        this.dTObstacles = new ArrayList<>();
        this.dTObstacles.add(this);
        this.dBObstacles = new ArrayList<>();
        this.dBObstacles.add(this);

        this.oo_distMin = new HashMap<>();
        this.oo_distXY = new HashMap<>();
        this.oBase_distMin = new HashMap<>();
        this.oBase_distXY = new HashMap<>();


    }

    public Map<PseudoBase, Map<PseudoBase, Integer>> getoBase_distMin() {
        return oBase_distMin;
    }

    public void addTo_oBase_distMin(PseudoBase base, Map<PseudoBase, Integer> dist) {
        this.oBase_distMin.put(base, dist);
    }

    public Map<PseudoBase, Map<PseudoBase, Integer>> getoBase_distXY() {
        return oBase_distXY;
    }

    public void addTo_oBase_distXY(PseudoBase base, Map<PseudoBase, Integer> dist) {
        this.oBase_distXY.put(base, dist);
    }

    public Map<Obstacle, int[]> getOo_distMin() {
        return oo_distMin;
    }

    public void addTo_oo_distMin(Obstacle o, int[] dist) {
        this.oo_distMin.put(o, dist);
    }

    public Map<Obstacle, int[]> getOo_distXY() {
        return oo_distXY;
    }

    public void addTo_oo_distXY(Obstacle o, int[] dist) {
        this.oo_distXY.put(o, dist);
    }

    public ArrayList<PseudoBase> getBaseArray() {
        return baseArray;
    }

    public double getDelta_x() {
        return delta_x;
    }

    public double getDelta_y() {
        return delta_y;
    }

    public double getGradient() {
        return gradient;
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

    public ArrayList<Obstacle> get_tLObstacles() {
        return tLObstacles;
    }

    public void addTotLObstacles(Obstacle o) {
        this.tLObstacles.add(o);
    }

    public ArrayList<Obstacle> get_tRObstacles() {
        return tRObstacles;
    }

    public void addTotRObstacles(Obstacle o) {
        this.tRObstacles.add(o);
    }

    public ArrayList<Obstacle> get_bLObstacles() {
        return bLObstacles;
    }

    public void addTobLObstacles(Obstacle o) {
        this.bLObstacles.add(o);
    }

    public ArrayList<Obstacle> get_bRObstacles() {
        return bRObstacles;
    }

    public void addTobRObstacles(Obstacle o) {
        this.bRObstacles.add(o);
    }

    public ArrayList<Obstacle> get_dLObstacles() {
        return dLObstacles;
    }

    public void addTodLObstacles(Obstacle o) {
        this.dLObstacles.add(o);
    }

    public ArrayList<Obstacle> get_dRObstacles() {
        return dRObstacles;
    }

    public void addTodRObstacles(Obstacle o) {
        this.dRObstacles.add(o);
    }

    public ArrayList<Obstacle> get_dTObstacles() {
        return dTObstacles;
    }

    public void addTodTObstacles(Obstacle o) {
        this.dTObstacles.add(o);
    }

    public ArrayList<Obstacle> get_dBObstacles() {
        return dBObstacles;
    }

    public void addTodBObstacles(Obstacle o) {
        this.dBObstacles.add(o);
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

    public boolean aboveUp(PseudoBase base) {
        return base.getY() > this.gradient * (base.getX() - this.minX) + this.minY;
    }

    public boolean belowUp(PseudoBase base) {
        return base.getY() < this.gradient * (base.getX() - this.minX) + this.minY;
    }

    public boolean aboveDown(PseudoBase base) {
        return base.getY() > -this.gradient * (base.getX() - this.minX) + this.maxY;
    }

    public boolean belowDown(PseudoBase base) {
        return base.getY() < -this.gradient * (base.getX() - this.minX) + this.maxY;
    }

    public boolean down_AreaOverlap(Obstacle other_o) {
        return other_o.minY + other_o.minX <= this.maxY + this.maxX && other_o.maxY + other_o.maxX >= this.minY + this.minX;
    }

    public boolean up_AreaOverlap(Obstacle other_o) {
        return other_o.maxY - other_o.minX > this.minY - this.maxX && other_o.minY - other_o.maxX < this.maxY - this.minX;
    }

    //Conference 45-degree Set
    public boolean atL(Obstacle other_o) {
        return down_AreaOverlap(other_o) && this.aboveUp(other_o.getLowerRight());
    }

    public boolean abR(Obstacle other_o) {
        return down_AreaOverlap(other_o) && this.belowUp(other_o.getLowerRight());
    }

    public boolean atR(Obstacle other_o) {
        return up_AreaOverlap(other_o) && this.aboveDown(other_o.getUpperRight());
    }

    public boolean abL(Obstacle other_o) {
        return up_AreaOverlap(other_o) && this.belowDown(other_o.getUpperRight());
    }

    //Conference rectangular Set
    public boolean below(Obstacle on) {
        return on.minY >= this.maxY;
    }

    public boolean above(Obstacle on) {
        return on.maxY <= this.minY;
    }

    public boolean right(Obstacle on) {
        return on.maxX <= this.minX;
    }

    public boolean left(Obstacle on) {
        return on.minX >= this.maxX;
    }


    public boolean odL(Obstacle on) {
        return !this.below(on) && !this.above(on) && this.right(on);
    }

    public boolean odR(Obstacle on) {
        return !this.below(on) && !this.above(on) && this.left(on);
    }

    public boolean odT(Obstacle on) {
        return !this.left(on) && !this.right(on) && this.below(on);
    }

    public boolean odB(Obstacle on) {
        return !this.left(on) && !this.right(on) && this.above(on);
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
