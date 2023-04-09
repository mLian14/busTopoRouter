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


    private Map<Obstacle,  Map<BaseType, ArrayList<Obstacle>>> mapLowerLeftBypassP2Os;
    private Map<Obstacle,  Map<BaseType, ArrayList<Obstacle>>> mapLowerRightBypassP2Os;
    private Map<Obstacle,  Map<BaseType, ArrayList<Obstacle>>> mapUpperLeftBypassP2Os;
    private Map<Obstacle,  Map<BaseType, ArrayList<Obstacle>>> mapUpperRightBypassP2Os;

    private Map<Obstacle,  Map<BaseType, ArrayList<Obstacle>>> mapLowerLeftBypassP4Os;
    private Map<Obstacle,  Map<BaseType, ArrayList<Obstacle>>> mapLowerRightBypassP4Os;
    private Map<Obstacle,  Map<BaseType, ArrayList<Obstacle>>> mapUpperLeftBypassP4Os;
    private Map<Obstacle,  Map<BaseType, ArrayList<Obstacle>>> mapUpperRightBypassP4Os;

    private ArrayList<Map<Obstacle,  Map<BaseType, ArrayList<Obstacle>>>> bypassOsMapArray;//LL, LR, UL, UR

    private Map<Obstacle, Map<BaseType, Double>> mapLowerLeftL;
    private Map<Obstacle, Map<BaseType, Double>> mapLowerRightL;
    private Map<Obstacle, Map<BaseType, Double>> mapUpperLeftL;
    private Map<Obstacle, Map<BaseType, Double>> mapUpperRightL;

    private ArrayList<Map<Obstacle, Map<BaseType, Double>>> lengthMapArray;//LL, LR, UL, UR


    private Map<Obstacle, Map<BaseType, Path>> mapLowerLeftPath;
    private Map<Obstacle, Map<BaseType, Path>> mapLowerRightPath;
    private Map<Obstacle, Map<BaseType, Path>> mapUpperLeftPath;
    private Map<Obstacle, Map<BaseType, Path>> mapUpperRightPath;

    private ArrayList<Map<Obstacle, Map<BaseType, Path>>> pathMapArray;//LL, LR, UL, UR

    private ArrayList<Obstacle> topL_bottomR_Os, bottomL_topR_Os;



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



        this.mapLowerLeftBypassP2Os = new HashMap<>();
        this.mapLowerRightBypassP2Os = new HashMap<>();
        this.mapUpperLeftBypassP2Os = new HashMap<>();
        this.mapUpperRightBypassP2Os = new HashMap<>();
        this.mapLowerLeftBypassP4Os = new HashMap<>();
        this.mapLowerRightBypassP4Os = new HashMap<>();
        this.mapUpperLeftBypassP4Os = new HashMap<>();
        this.mapUpperRightBypassP4Os = new HashMap<>();
        this.bypassOsMapArray = new ArrayList<>(Arrays.asList(this.mapLowerLeftBypassP2Os, this.mapLowerRightBypassP2Os, this.mapUpperLeftBypassP2Os, this.mapUpperRightBypassP2Os, this.mapLowerLeftBypassP4Os, this.mapLowerRightBypassP4Os, this.mapUpperLeftBypassP4Os, this.mapUpperRightBypassP4Os));

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

        this.topL_bottomR_Os = new ArrayList<>();
        this.bottomL_topR_Os = new ArrayList<>();

    }

    public ArrayList<PseudoBase> getBaseArray() {
        return baseArray;
    }

    public ArrayList<Obstacle> getTopL_bottomR_Os() {
        return topL_bottomR_Os;
    }

    public void addTo_topL_bottomR_Os(Obstacle o) {
        this.topL_bottomR_Os.add(o);
    }

    public ArrayList<Obstacle> getBottomL_topR_Os() {
        return bottomL_topR_Os;
    }

    public void addTo_bottomL_topR_Os(Obstacle o) {
        this.bottomL_topR_Os.add(o);
    }


    public ArrayList<Map<Obstacle, Map<BaseType, ArrayList<Obstacle>>>> getBypassOsMapArray() {
        return bypassOsMapArray;
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


    public Map<Obstacle, Map<BaseType, ArrayList<Obstacle>>> getMapLowerLeftBypassP2Os() {
        return mapLowerLeftBypassP2Os;
    }

    public void addToMapLowerLeftBypassOs(Obstacle o, Map<BaseType, ArrayList<Obstacle>> map) {
        this.mapLowerLeftBypassP2Os.put(o, map);
    }


    public Map<Obstacle, Map<BaseType, ArrayList<Obstacle>>> getMapLowerRightBypassP2Os() {
        return mapLowerRightBypassP2Os;
    }

    public void addToMapLowerRightBypassOs(Obstacle o, Map<BaseType, ArrayList<Obstacle>> map) {
        this.mapLowerRightBypassP2Os.put(o, map);
    }

    public Map<Obstacle, Map<BaseType, ArrayList<Obstacle>>> getMapUpperLeftBypassP2Os() {
        return mapUpperLeftBypassP2Os;
    }

    public void addToMapUpperLeftBypassOs(Obstacle o, Map<BaseType, ArrayList<Obstacle>> map) {
        this.mapUpperLeftBypassP2Os.put(o, map);
    }

    public Map<Obstacle, Map<BaseType, ArrayList<Obstacle>>> getMapUpperRightBypassP2Os() {
        return mapUpperRightBypassP2Os;
    }

    public void addToMapUpperRightBypassOs(Obstacle o, Map<BaseType, ArrayList<Obstacle>> map) {
        this.mapUpperRightBypassP2Os.put(o, map);
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



    public Map<Obstacle, Map<BaseType, Path>> getMapLowerRightPath() {
        return mapLowerRightPath;
    }


    public Map<Obstacle, Map<BaseType, Path>> getMapUpperLeftPath() {
        return mapUpperLeftPath;
    }


    public Map<Obstacle, Map<BaseType, Path>> getMapUpperRightPath() {
        return mapUpperRightPath;
    }


    public void addToPathMap(int cnt, Obstacle o, Map<BaseType, Path> map) {
        this.pathMapArray.get(cnt).put(o, map);
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

    public boolean topL_bottomR_AreaOverlap(Obstacle other_o){
        return other_o.minY + other_o.minX <= this.maxY + this.maxX && other_o.maxY + other_o.maxX >= this.minY + this.minX;
    }

    public boolean bottomL_topR_AreaOverlap(Obstacle other_o){
        return other_o.minY - other_o.maxX <= this.maxY - this.maxX && other_o.maxY - other_o.minX >= this.minY - this.maxX;
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
                //"LowerLeft_Os = " + convertMapMapOArrayToString(mapLowerLeftPathAreaOs) +
                //"LowerRight_Os = " + convertMapMapOArrayToString(mapLowerRightPathAreaOs) +
                "UpperLeft_Os = " + convertMapMapOArrayToString(mapUpperLeftBypassP2Os) +
                //"UpperRight_Os = " + convertMapMapOArrayToString(mapUpperRightPathAreaOs) +
                "LowerLeft_Length = " + convertMapMapDToString(mapLowerLeftL) +
                "LowerRight_Length = " + convertMapMapDToString(mapLowerRightL) +
                "UpperLeft_Length = " + convertMapMapDToString(mapUpperLeftL) +
                "UpperRight_Length = " + convertMapMapDToString(mapUpperRightL) +
                '}';
    }
}
