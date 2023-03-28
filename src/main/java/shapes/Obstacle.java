package shapes;

import org.apache.commons.lang3.tuple.Pair;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;


/**
 * @auther lianmeng
 * @create 25.03.23
 */
public class Obstacle {

    private String name;

    private int minX, maxX, minY, maxY;

    private PseudoBase lowerLeft, lowerRight, upperLeft, upperRight;

    private Map<Obstacle, Pair<BaseType, Integer>[]> map_oo_corner_dist;

    private Map<Obstacle,  Pair<BaseType, ArrayList<Obstacle>>[]> mapLowerLeftPathAreaOs;
    private Map<Obstacle,  Pair<BaseType, ArrayList<Obstacle>>[]> mapLowerRightPathAreaOs;
    private Map<Obstacle,  Pair<BaseType, ArrayList<Obstacle>>[]> mapUpperLeftPathAreaOs;
    private Map<Obstacle,  Pair<BaseType, ArrayList<Obstacle>>[]> mapUpperRightPathAreaOs;


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


        this.map_oo_corner_dist = new HashMap<>();
        this.mapLowerLeftPathAreaOs = new HashMap<>();
        this.mapLowerRightPathAreaOs = new HashMap<>();
        this.mapUpperLeftPathAreaOs = new HashMap<>();
        this.mapUpperRightPathAreaOs = new HashMap<>();
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

    public Map<Obstacle, Pair<BaseType, Integer>[]> getMap_oo_corner_dist() {
        return map_oo_corner_dist;
    }

    public void addToMap_oo_corner_dist(Obstacle o, Pair<BaseType, Integer>[] distArray) {
        this.map_oo_corner_dist.put(o, distArray);
    }

    public Map<Obstacle, Pair<BaseType, ArrayList<Obstacle>>[]> getMapLowerLeftPathAreaOs() {
        return mapLowerLeftPathAreaOs;
    }

    public void addToMapLowerLeftPathAreaOs(Obstacle o, Pair<BaseType, ArrayList<Obstacle>>[] array) {
        this.mapLowerLeftPathAreaOs.put(o, array);
    }

    public Map<Obstacle, Pair<BaseType, ArrayList<Obstacle>>[]> getMapLowerRightPathAreaOs() {
        return mapLowerRightPathAreaOs;
    }

    public void addToMapLowerRightPathAreaOs(Obstacle o, Pair<BaseType, ArrayList<Obstacle>>[] array) {
        this.mapLowerRightPathAreaOs.put(o, array);
    }

    public Map<Obstacle, Pair<BaseType, ArrayList<Obstacle>>[]> getMapUpperLeftPathAreaOs() {
        return mapUpperLeftPathAreaOs;
    }

    public void addToMapUpperLeftPathAreaOs(Obstacle o, Pair<BaseType, ArrayList<Obstacle>>[] array) {
        this.mapUpperLeftPathAreaOs.put(o, array);
    }

    public Map<Obstacle, Pair<BaseType, ArrayList<Obstacle>>[]> getMapUpperRightPathAreaOs() {
        return mapUpperRightPathAreaOs;
    }

    public void addToMapUpperRightPathAreaOs(Obstacle o, Pair<BaseType, ArrayList<Obstacle>>[] array) {
        this.mapUpperRightPathAreaOs.put(o, array);
    }

    public String convertMapPairToString(Map<Obstacle, Pair<BaseType, Integer>[]> mapPair) {

        StringBuilder mapAsString = new StringBuilder("||");
        for (Obstacle o : mapPair.keySet()) {
            Pair<BaseType, Integer>[] distArray = mapPair.get(o);
            for (int i = 0; i < distArray.length; ++i) {
                mapAsString.append(this.getName() + "-" + o.getName() + "." + distArray[i].getKey().toString() + "-" + distArray[i].getValue() + "\n");
            }
        }
        mapAsString.append("||");
        return mapAsString.toString();
    }


    @Override
    public String toString() {
        return "Obstacle{" +
                "name='" + name + '\'' +
                ", minX=" + minX +
                ", maxX=" + maxX +
                ", minY=" + minY +
                ", maxY=" + maxY + "\n" +
                ", mapOOCornerDist = " + convertMapPairToString(map_oo_corner_dist) +
                '}';
    }
}
