da1
da2
da3
da4

package da5.da6.da7.da8;

import da9.da10.da11.da12.da13.da14;
import da9.da10.da11.da12.da13.da15;
import da9.da10.da11.da12.da13.da16;
import da9.da10.da11.da12.da17.da18;
import da9.da10.da11.da19.da20;
import da9.da10.da11.da19.da21;
import da9.da10.da11.da19.da22;
import da9.da10.da11.da19.da23;
import da9.da10.da11.da24.da25;
import da26.da17.da27;
import da26.da17.da28;
import da26.da17.da29;
import da26.da17.da30;
import da31.da32.da33.da34;

public class da35 {
  private final da20 da36;

  private final da22 da37;
  private final da22 da38;
  private final da22 da39;
  private final da21 da40;
  private final da21 da41;

  public da35(da42 da43) {
    da44 da45 = da23.da46().da47(da43);
    da36 = da45.da48(da49).da50();
    da37 = da45.da51(da52).da53(0.0);
    da38 = da45.da51(da54).da53(0.0);
    da39 = da45.da51(da55).da53(0.0);
    da40 = da45.da48(da56).da53(new double[] {});
    da41 =
        da45.da48(da57).da53(new double[] {});
  }

  public void da58(da59 da60, double da61) {
    da62
    da63
    da60.da64 =
        ((da25.da65() - da37.da66()) / 1e3) < 250;

    da67
    da60.da68 =
        new da69(
            da15.da70(da38.da71()), da15.da70(da39.da71()));

    da72
    da36.da73(new double[] {da61, 0.0, 0.0, 0.0, 0.0, 0.0});
    da23.da46()
        .da74(); da75

    da76
    da30<da77> da78 = new da27<>();
    da29<da79> da80 = new da28<>();

    da81
    for (da44 da82 : da40.da83()) {
      if (da82.da84.da85 == 0) continue;
      for (int da86 = 11; da86 < da82.da84.da85; da86 += 7) {
        da78.da87((int) da82.da84[da86]);
      }

      da80.da87(
          new da79(
              da88
              da82.da89 * 1.0e-3 - da82.da84[6],

              da90
              da91(da82.da84),

              da92
              da93
              da82.da84.da85 >= 18 ? da82.da84[17] : 0.0,

              da94
              (int) da82.da84[7],

              da95
              da82.da84[9],

              da96
              da97.da98));
    }

    da99
    for (da44 da82 : da41.da83()) {
      if (da82.da84.da85 == 0) continue;
      for (int da86 = 11; da86 < da82.da84.da85; da86 += 7) {
        da78.da87((int) da82.da84[da86]);
      }
      da80.da87(
          new da79(
              da100
              da82.da89 * 1.0e-3 - da82.da84[6],

              da101
              da91(da82.da84),

              da102
              0.0,

              da103
              (int) da82.da84[7],

              da104
              da82.da84[9],

              da105
              da97.da98));
    }

    da106
    da60.da80 = new da79[da80.da107()];
    for (int da86 = 0; da86 < da80.da107(); da86++) {
      da60.da80[da86] = da80.da71(da86);
    }

    da108
    da60.da78 = new int[da78.da107()];
    int da86 = 0;
    for (int da109 : da78) {
      da60.da78[da86++] = da109;
    }
  }

  da110
  private static da14 da91(double[] da111) {
    return new da14(
        da111[0],
        da111[1],
        da111[2],
        new da16(
            da18.da112(da111[3]),
            da18.da112(da111[4]),
            da18.da112(da111[5])));
  }

  @da34
  public static class da59 {
    public boolean da64 = false;
    public da69 da68 =
        new da69(new da15(), new da15());
    public da79[] da80 = new da79[0];
    public int[] da78 = new int[0];
  }

  da113
  public static da114 da69(da15 da115, da15 da116) {}

  public static enum da97 {
    da98,
    da117,
  }

  da118
  public static da114 da79(
      double da89,
      da14 da119,
      double da120,
      int da121,
      double da122,
      da97 da123) {}
}
