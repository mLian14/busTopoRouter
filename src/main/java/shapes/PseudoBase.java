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
    private Map<Obstacle, int[]> pseudo_oDir_qs;

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
    private Map<Obstacle, int[]> pseudo_oRel_qs;


    /*

     */
    private Map<Obstacle, int[]> pseudo_iVars;

    private ArrayList<Obstacle> oLd;
    private ArrayList<Obstacle> oRd;
    private ArrayList<Obstacle> oAd;
    private ArrayList<Obstacle> oBd;
    private ArrayList<Obstacle> oULd;
    private ArrayList<Obstacle> oURd;
    private ArrayList<Obstacle> oLLd;
    private ArrayList<Obstacle> oLRd;

    /*
    0: L
    1: R
    2: Top
    3: Bottom

     */
    private Map<PseudoBase, int[]> pseudo_pDir_qs;

    /*
    0: dL-R
    1: dR-L
    2: dT-B
    3: dB-T
    4: UL-LR
    5: LR-UL
    6: UR-LL
    7: LL-UR
     */
    private Map<PseudoBase, int[]> pseudo_pRel_qs;



    public PseudoBase(int x, int y) {
        this.x = x;
        this.y = y;
        this.pseudo_oDir_qs = new HashMap<>();
        this.pseudo_oRel_qs = new HashMap<>();
        this.pseudo_iVars = new HashMap<>();

        this.pseudo_pDir_qs = new HashMap<>();

        this.oLd = new ArrayList<>();
        this.oRd = new ArrayList<>();
        this.oAd = new ArrayList<>();
        this.oBd = new ArrayList<>();
        this.oULd = new ArrayList<>();
        this.oURd = new ArrayList<>();
        this.oLLd = new ArrayList<>();
        this.oLRd = new ArrayList<>();


    }

    public PseudoBase() {

        this.pseudo_oDir_qs = new HashMap<>();
        this.pseudo_oRel_qs = new HashMap<>();
        this.pseudo_iVars = new HashMap<>();

        this.oLd = new ArrayList<>();
        this.oRd = new ArrayList<>();
        this.oAd = new ArrayList<>();
        this.oBd = new ArrayList<>();
        this.oULd = new ArrayList<>();
        this.oURd = new ArrayList<>();
        this.oLLd = new ArrayList<>();
        this.oLRd = new ArrayList<>();
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

    public Map<Obstacle, int[]> getPseudo_oDir_qs() {
        return pseudo_oDir_qs;
    }

    public void addToPseudo_oDir_qs(Obstacle o, int[] q) {
        this.pseudo_oDir_qs.put(o, q);
    }

    public Map<Obstacle, int[]> getPseudo_oRel_qs() {
        return pseudo_oRel_qs;
    }

    public void addToPseudo_oRel_qs(Obstacle o, int[] q) {
        this.pseudo_oRel_qs.put(o, q);
    }

    public Map<PseudoBase, int[]> getPseudo_pDir_qs() {
        return pseudo_pDir_qs;
    }

    public void addToPseudo_pDir_qs(PseudoBase base, int[] q) {
        this.pseudo_pDir_qs.put(base, q);
    }

    public ArrayList<Obstacle> getOLd() {
        return oLd;
    }

    public void addToOLd(Obstacle o) {
        this.oLd.add(o);
    }

    public ArrayList<Obstacle> getORd() {
        return oRd;
    }

    public void addToORd(Obstacle o) {
        this.oRd.add(o);
    }

    public ArrayList<Obstacle> getOAd() {
        return oAd;
    }

    public void addToOAd(Obstacle o) {
        this.oAd.add(o);
    }

    public ArrayList<Obstacle> getOBd() {
        return oBd;
    }

    public void addToOBd(Obstacle o) {
        this.oBd.add(o);
    }

    public ArrayList<Obstacle> getOULd() {
        return oULd;
    }

    public void addToOULd(Obstacle o) {
        this.oULd.add(o);
    }

    public ArrayList<Obstacle> getOURd() {
        return oURd;
    }

    public void addToOURd(Obstacle o) {
        this.oURd.add(o);
    }

    public ArrayList<Obstacle> getOLLd() {
        return oLLd;
    }

    public void addToOLLd(Obstacle o) {
        this.oLLd.add(o);
    }

    public ArrayList<Obstacle> getOLRd() {
        return oLRd;
    }

    public void addToOLRd(Obstacle o) {
        this.oLRd.add(o);
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (o == null || getClass() != o.getClass()) return false;
        PseudoBase point = (PseudoBase) o;
        return Double.compare(point.x, x) == 0 && Double.compare(point.y, y) == 0;
    }

    public String convertMapIntArrayToString(Map<Obstacle, int[]> mapIntArray) {
        StringBuilder mapAsString = new StringBuilder("||");
        for (Obstacle o : mapIntArray.keySet()) {
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
                ", mapQ = " + convertMapIntArrayToString(pseudo_oDir_qs) +
                ", mapIq = " + convertMapIntArrayToString(pseudo_iVars) +
                '\'' +
                '}';
    }


}

