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


    private Map<Obstacle,  Map<BaseType, ArrayList<Obstacle>>> mapLowerLeftBypassOs;
    private Map<Obstacle,  Map<BaseType, ArrayList<Obstacle>>> mapLowerRightBypassOs;
    private Map<Obstacle,  Map<BaseType, ArrayList<Obstacle>>> mapUpperLeftBypassOs;
    private Map<Obstacle,  Map<BaseType, ArrayList<Obstacle>>> mapUpperRightBypassOs;

    private ArrayList<Map<Obstacle,  Map<BaseType, ArrayList<Obstacle>>>> bypassMapArray;

    private Map<Obstacle, Map<BaseType, Double>> mapLowerLeftL;
    private Map<Obstacle, Map<BaseType, Double>> mapLowerRightL;
    private Map<Obstacle, Map<BaseType, Double>> mapUpperLeftL;
    private Map<Obstacle, Map<BaseType, Double>> mapUpperRightL;

    private ArrayList<Map<Obstacle, Map<BaseType, Double>>> lengthMapArray;


    private Map<Obstacle, Map<BaseType, Path>> mapLowerLeftPath;
    private Map<Obstacle, Map<BaseType, Path>> mapLowerRightPath;
    private Map<Obstacle, Map<BaseType, Path>> mapUpperLeftPath;
    private Map<Obstacle, Map<BaseType, Path>> mapUpperRightPath;

    private ArrayList<Map<Obstacle, Map<BaseType, Path>>> pathMapArray;

    private ArrayList<Obstacle> rightOs, leftOs, topOs, bottomOs;
    private ArrayList<Obstacle> topLeftOs, bottomLeftOs, topRightOs, bottomRightOs;



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



        this.mapLowerLeftBypassOs = new HashMap<>();
        this.mapLowerRightBypassOs = new HashMap<>();
        this.mapUpperLeftBypassOs = new HashMap<>();
        this.mapUpperRightBypassOs = new HashMap<>();
        this.bypassMapArray = new ArrayList<>(Arrays.asList(this.mapLowerLeftBypassOs, this.mapLowerRightBypassOs, this.mapUpperLeftBypassOs, this.mapUpperRightBypassOs));

        this.mapLowerLeftL = new HashMap<>();
        this.mapLowerRightL = new HashMap<>();
        this.mapUpperLeftL = new HashMap<>();
        this.mapUpperRightL = new HashMap<>();
        this.lengthMapArray = new ArrayList<>(Arrays.asList(this.mapLowerLeftL, this.mapLowerRightL, this.mapUpperLeftL, this.mapUpperRightL));

        this.mapLowerLeftPath = new HashMap<>();
        this.mapLowerRightPath = new HashMap<>();
        this.mapUpperLeftPath = new HashMap<>();
        this.mapUpperRightPath = new HashMap<>();
        this.pathMapArray = new ArrayList<>(Arrays.asList(this.mapLowerLeftPath, this.mapLowerRightPath, this.mapUpperLeftPath, this.mapUpperRightPath));

        this.leftOs = new ArrayList<>();
        this.rightOs = new ArrayList<>();
        this.topOs = new ArrayList<>();
        this.bottomOs = new ArrayList<>();
        this.topLeftOs = new ArrayList<>();
        this.bottomLeftOs = new ArrayList<>();
        this.topRightOs = new ArrayList<>();
        this.bottomRightOs = new ArrayList<>();
    }

    public ArrayList<Obstacle> getRightOs() {
        return rightOs;
    }

    public void addToRightOs(Obstacle o) {
        this.rightOs.add(o);
    }

    public ArrayList<Obstacle> getLeftOs() {
        return leftOs;
    }

    public void addToLeftOs(Obstacle o) {
        this.leftOs.add(o);
    }

    public ArrayList<Obstacle> getTopOs() {
        return topOs;
    }

    public void addToTopOs(Obstacle o) {
        this.topOs.add(o);
    }

    public ArrayList<Obstacle> getBottomOs() {
        return bottomOs;
    }

    public void addToBottomOs(Obstacle o) {
        this.bottomOs.add(o);
    }

    public ArrayList<Obstacle> getTopLeftOs() {
        return topLeftOs;
    }

    public void addToTopLeftOs(Obstacle o) {
        this.topLeftOs.add(o);
    }

    public ArrayList<Obstacle> getBottomLeftOs() {
        return bottomLeftOs;
    }

    public void addToBottomLeftOs(Obstacle o) {
        this.bottomLeftOs.add(o);
    }

    public ArrayList<Obstacle> getTopRightOs() {
        return topRightOs;
    }

    public void addToTopRightOs(Obstacle o) {
        this.topRightOs.add(o);
    }

    public ArrayList<Obstacle> getBottomRightOs() {
        return bottomRightOs;
    }

    public void addToBottomRightOs(Obstacle o) {
        this.bottomRightOs.add(o);
    }

    public ArrayList<Map<Obstacle, Map<BaseType, ArrayList<Obstacle>>>> getBypassMapArray() {
        return bypassMapArray;
    }

    public ArrayList<Map<Obstacle, Map<BaseType, Double>>> getLengthMapArray() {
        return lengthMapArray;
    }

    public ArrayList<Map<Obstacle, Map<BaseType, Path>>> getPathMapArray() {
        return pathMapArray;
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


    public Map<Obstacle, Map<BaseType, ArrayList<Obstacle>>> getMapLowerLeftBypassOs() {
        return mapLowerLeftBypassOs;
    }

    public void addToMapLowerLeftBypassOs(Obstacle o, Map<BaseType, ArrayList<Obstacle>> map) {
        this.mapLowerLeftBypassOs.put(o, map);
    }

    public Map<Obstacle, Map<BaseType, ArrayList<Obstacle>>> getMapLowerRightBypassOs() {
        return mapLowerRightBypassOs;
    }

    public void addToMapLowerRightBypassOs(Obstacle o, Map<BaseType, ArrayList<Obstacle>> map) {
        this.mapLowerRightBypassOs.put(o, map);
    }

    public Map<Obstacle, Map<BaseType, ArrayList<Obstacle>>> getMapUpperLeftBypassOs() {
        return mapUpperLeftBypassOs;
    }

    public void addToMapUpperLeftBypassOs(Obstacle o, Map<BaseType, ArrayList<Obstacle>> map) {
        this.mapUpperLeftBypassOs.put(o, map);
    }

    public Map<Obstacle, Map<BaseType, ArrayList<Obstacle>>> getMapUpperRightBypassOs() {
        return mapUpperRightBypassOs;
    }

    public void addToMapUpperRightBypassOs(Obstacle o, Map<BaseType, ArrayList<Obstacle>> map) {
        this.mapUpperRightBypassOs.put(o, map);
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

    public Map<Obstacle, Map<BaseType, Path>> getMapLowerLeftPath() {
        return mapLowerLeftPath;
    }

    public void addToMapLowerLeftPath(Obstacle o, Map<BaseType, Path> map) {
        this.mapLowerLeftPath.put(o, map);
    }

    public Map<Obstacle, Map<BaseType, Path>> getMapLowerRightPath() {
        return mapLowerRightPath;
    }

    public void addToMapLowerRightPath(Obstacle o, Map<BaseType, Path> map) {
        this.mapLowerRightPath.put(o, map);
    }

    public Map<Obstacle, Map<BaseType, Path>> getMapUpperLeftPath() {
        return mapUpperLeftPath;
    }

    public void addToMapUpperLeftPath(Obstacle o, Map<BaseType, Path> map) {
        this.mapUpperLeftPath.put(o, map);
    }

    public Map<Obstacle, Map<BaseType, Path>> getMapUpperRightPath() {
        return mapUpperRightPath;
    }

    public void addToMapUpperRightPath(Obstacle o, Map<BaseType, Path> map) {
        this.mapUpperRightPath.put(o, map);
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
                //"LowerLeft_Os = " + convertMapMapOArrayToString(mapLowerLeftPathAreaOs) +
                //"LowerRight_Os = " + convertMapMapOArrayToString(mapLowerRightPathAreaOs) +
                "UpperLeft_Os = " + convertMapMapOArrayToString(mapUpperLeftBypassOs) +
                //"UpperRight_Os = " + convertMapMapOArrayToString(mapUpperRightPathAreaOs) +
                "LowerLeft_Length = " + convertMapMapDToString(mapLowerLeftL) +
                "LowerRight_Length = " + convertMapMapDToString(mapLowerRightL) +
                "UpperLeft_Length = " + convertMapMapDToString(mapUpperLeftL) +
                "UpperRight_Length = " + convertMapMapDToString(mapUpperRightL) +
                '}';
    }
}
