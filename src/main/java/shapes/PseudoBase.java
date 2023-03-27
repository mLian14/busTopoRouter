package shapes;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;

/**
 * @auther lianmeng
 * @create 25.03.23
 */
public class PseudoBase {

    private int x, y;
    private String name;
    private BaseType type;

    /*
    0: oqL
    1: oqR
    2: oqA
    3: oqB
    4: oqUL
    5: oqUR
    6: oqLR
    7: oqLL
    8: oqD
    9: oqU

     */
    private Map<Obstacle, int[]> pseudo_Dir_qs;

    /*
    0: oqLd
    1: oqRd
    2: oqAd
    3: oqBd
    4: oqUL
    5: oqUR
    6: oqLL
    7: oqLR
     */
    private Map<Obstacle, int[]> pseudo_Rel_qs;


    /*

     */
    private Map<Obstacle, int[]> pseudo_iVars;



    public PseudoBase(int x, int y) {
        this.x = x;
        this.y = y;
        this.pseudo_Dir_qs = new HashMap<>();
        this.pseudo_Rel_qs = new HashMap<>();
        this.pseudo_iVars = new HashMap<>();

    }

    public PseudoBase() {

        this.pseudo_Dir_qs = new HashMap<>();
        this.pseudo_Rel_qs = new HashMap<>();
        this.pseudo_iVars = new HashMap<>();
    }

    public BaseType getType() {
        return type;
    }

    public void setType(BaseType type) {
        this.type = type;
    }

    public String getName() {
        return name;
    }

    public void setName(String name) {
        this.name = name;
    }

    public int getX() {
        return x;
    }

    public void setX(int x) {
        this.x = x;
    }

    public int getY() {
        return y;
    }

    public void setY(int y) {
        this.y = y;
    }

    public Map<Obstacle, int[]> getPseudo_Dir_qs() {
        return pseudo_Dir_qs;
    }

    public void addToPseudo_Dir_qs(Obstacle o, int[] q) {
        this.pseudo_Dir_qs.put(o, q);
    }

    public Map<Obstacle, int[]> getPseudo_Rel_qs() {
        return pseudo_Rel_qs;
    }

    public void addToPseudo_Rel_qs(Obstacle o, int[] q) {
        this.pseudo_Rel_qs.put(o,q);
    }



    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (o == null || getClass() != o.getClass()) return false;
        PseudoBase point = (PseudoBase) o;
        return Double.compare(point.x, x) == 0 && Double.compare(point.y, y) == 0;
    }

    public String convertMapIntArrayToString(Map<Obstacle, int[]> mapIntArray){
        StringBuilder mapAsString = new StringBuilder("||");
        for (Obstacle o : mapIntArray.keySet()){
            mapAsString.append(o.getName() + "[" + Arrays.toString(mapIntArray.get(o)) + "];");
        }
        mapAsString.append("||");
        return mapAsString.toString();
    }


    @Override
    public String toString() {
        ArrayList<Integer> tmpQ = new ArrayList<>();


        return "PseudoBase{" +
                "x=" + x +
                ", y=" + y +
                ", name='" + name +
                ", mapQ = " + convertMapIntArrayToString(pseudo_Dir_qs) +
                ", mapIq = " + convertMapIntArrayToString(pseudo_iVars) +
                '\'' +
                '}';
    }



}

