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


    private Map<Obstacle,  Map<BaseType, ArrayList<Obstacle>>> mapLowerLeftPathAreaOs;
    private Map<Obstacle,  Map<BaseType, ArrayList<Obstacle>>> mapLowerRightPathAreaOs;
    private Map<Obstacle,  Map<BaseType, ArrayList<Obstacle>>> mapUpperLeftPathAreaOs;
    private Map<Obstacle,  Map<BaseType, ArrayList<Obstacle>>> mapUpperRightPathAreaOs;

    private Map<Obstacle, Map<BaseType, Double>> mapLowerLeftL;
    private Map<Obstacle, Map<BaseType, Double>> mapLowerRightL;
    private Map<Obstacle, Map<BaseType, Double>> mapUpperLeftL;
    private Map<Obstacle, Map<BaseType, Double>> mapUpperRightL;


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



        this.mapLowerLeftPathAreaOs = new HashMap<>();
        this.mapLowerRightPathAreaOs = new HashMap<>();
        this.mapUpperLeftPathAreaOs = new HashMap<>();
        this.mapUpperRightPathAreaOs = new HashMap<>();

        this.mapLowerLeftL = new HashMap<>();
        this.mapLowerRightL = new HashMap<>();
        this.mapUpperLeftL = new HashMap<>();
        this.mapUpperRightL = new HashMap<>();
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


    public Map<Obstacle, Map<BaseType, ArrayList<Obstacle>>> getMapLowerLeftPathAreaOs() {
        return mapLowerLeftPathAreaOs;
    }

    public void addToMapLowerLeftPathAreaOs(Obstacle o, Map<BaseType, ArrayList<Obstacle>> map) {
        this.mapLowerLeftPathAreaOs.put(o, map);
    }

    public Map<Obstacle, Map<BaseType, ArrayList<Obstacle>>> getMapLowerRightPathAreaOs() {
        return mapLowerRightPathAreaOs;
    }

    public void addToMapLowerRightPathAreaOs(Obstacle o, Map<BaseType, ArrayList<Obstacle>> map) {
        this.mapLowerRightPathAreaOs.put(o, map);
    }

    public Map<Obstacle, Map<BaseType, ArrayList<Obstacle>>> getMapUpperLeftPathAreaOs() {
        return mapUpperLeftPathAreaOs;
    }

    public void addToMapUpperLeftPathAreaOs(Obstacle o, Map<BaseType, ArrayList<Obstacle>> map) {
        this.mapUpperLeftPathAreaOs.put(o, map);
    }

    public Map<Obstacle, Map<BaseType, ArrayList<Obstacle>>> getMapUpperRightPathAreaOs() {
        return mapUpperRightPathAreaOs;
    }

    public void addToMapUpperRightPathAreaOs(Obstacle o, Map<BaseType, ArrayList<Obstacle>> map) {
        this.mapUpperRightPathAreaOs.put(o, map);
    }

    public Map<Obstacle, Map<BaseType, Double>> getMapLowerLeftL() {
        return mapLowerLeftL;
    }

    public void addToMapLowerLeftL(Obstacle o, Map<BaseType, Double> map) {
        this.mapLowerLeftL.put(o, map);
    }

    public Map<Obstacle, Map<BaseType, Double>> getMapLowerRightL() {
        return mapLowerRightL;
    }

    public void addToMapLowerRightL(Obstacle o, Map<BaseType, Double> map) {
        this.mapLowerRightL.put(o, map);
    }

    public Map<Obstacle, Map<BaseType, Double>> getMapUpperLeftL() {
        return mapUpperLeftL;
    }

    public void addToMapUpperLeftL(Obstacle o, Map<BaseType, Double> map) {
        this.mapUpperLeftL.put(o, map);
    }

    public Map<Obstacle, Map<BaseType, Double>> getMapUpperRightL() {
        return mapUpperRightL;
    }

    public void addToMapUpperRightL(Obstacle o, Map<BaseType, Double> map) {
        this.mapUpperRightL.put(o, map);
    }

    public String convertMapMapOArrayToString(Map<Obstacle, Map<BaseType, ArrayList<Obstacle>>> map) {

        StringBuilder mapAsString = new StringBuilder("||");
        for (Obstacle o : map.keySet()) {
            Map<BaseType, ArrayList<Obstacle>> subMap = map.get(o);
            for (BaseType type : subMap.keySet()) {
                mapAsString.append(this.getName() + "-" + o.getName() + "." + type.toString() + "{" );
                for (Obstacle oo : subMap.get(type)){
                    mapAsString.append(oo.getName() + "; ");
                }
                mapAsString.append("}");
                mapAsString.append("\n");
            }
        }
        mapAsString.append("||");
        mapAsString.append("\n");
        return mapAsString.toString();
    }

    public String convertMapMapDToString(Map<Obstacle, Map<BaseType, Double>> map) {

        StringBuilder mapAsString = new StringBuilder("||");
        for (Obstacle o : map.keySet()) {
            Map<BaseType, Double> subMap = map.get(o);
            for (BaseType type : subMap.keySet()) {
                mapAsString.append(this.getName() + "-" + o.getName() + "." + type.toString() + "{" + subMap.get(type) + "}");
                mapAsString.append("\n");
            }
        }
        mapAsString.append("||");
        mapAsString.append("\n");
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
                //"LowerLeft_Os = " + convertMapMapOArrayToString(mapLowerLeftPathAreaOs) +
                //"LowerRight_Os = " + convertMapMapOArrayToString(mapLowerRightPathAreaOs) +
                //"UpperLeft_Os = " + convertMapMapOArrayToString(mapUpperLeftPathAreaOs) +
                //"UpperRight_Os = " + convertMapMapOArrayToString(mapUpperRightPathAreaOs) +
                "LowerLeft_Length = " + convertMapMapDToString(mapLowerLeftL) +
                "LowerRight_Length = " + convertMapMapDToString(mapLowerRightL) +
                "UpperLeft_Length = " + convertMapMapDToString(mapUpperLeftL) +
                "UpperRight_Length = " + convertMapMapDToString(mapUpperRightL) +
                '}';
    }
}
