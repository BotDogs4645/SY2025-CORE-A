da1
da2
da3
da4

package da5.da6.da7.da8;

import da9.da10.da11.da12.da13.da14;
import da15.da16.da17.da18.da19;
import da15.da16.da17.da18.da20;
import da15.da16.da17.da18.da21.da22;
import da15.da16.da17.da18.da21.da23;
import da15.da16.da17.da18.da24.da25;
import da15.da16.da17.da18.da24.da26;
import da15.da16.da17.da27.da28;
import da15.da16.da17.da27.da28.da29;
import da15.da16.da17.da30.da31.da32;
import da5.da6.da33.da34;
import da5.da6.da7.da8.da35.da36;
import da37.da38.da39;
import da37.da38.da40;
import da37.da38.da41.da42;
import da43.da44.da45.da46;

public class da47 extends da32 {

  private final da48 da49;
  private final da42<da14> da50;

  private final da35[] da51;
  private final da52[] da53;
  private final da28[] da54;

  public da47(da48 da49, da42<da14> da50) {
    this.da49 = da49;
    this.da50 = da50;

    this.da51 = new da35[da34.da55.da56];
    this.da53 = new da52[da34.da55.da56];
    this.da54 = new da28[da34.da55.da56];

    for (int da57 = 0; da57 < da34.da55.da56; da57++) {
      da51[da57] = new da35(da34.da55[da57]);
      da53[da57] = new da52();
      da54[da57] =
          new da28(
              da58 + da34.da55[da57] + da59,
              da29.da60);
    }
  }

  @da61
  public void da62() {
    da14 da63 = da50.da64();
    double da65 = da63.da66.da67().da68();

    da69
    da40<da23> da70 = new da39<>();
    da40<da23> da71 = new da39<>();
    da40<da23> da72 = new da39<>();
    da40<da23> da73 = new da39<>();

    for (int da57 = 0; da57 < da51.da56; da57++) {
      da51[da57].da74(da53[da57], da65);
      da46.da75(da76 + da34.da55[da57], da53[da57]);
      da54[da57].da77(!da53[da57].da78);

      da79
      da40<da23> da80 = new da39<>();
      da40<da23> da81 = new da39<>();
      da40<da23> da82 = new da39<>();
      da40<da23> da83 = new da39<>();

      da84
      for (int da85 : da53[da57].da86) {
        da87 da88 = da34.da89.da90(da85);
        da88.da91(da80::da92);
      }

      da93
      for (da87 da94 : da53[da57].da95) {
        da96
        boolean da97 =
            da94.da98() == 0
                ||
                da99
                da94.da100() > da34.da101
                || da102.da103(da94.da104().da105()) > da34.da106
                ||
                da107
                da94.da104().da108() < 0.0
                || da94.da104().da108() > da34.da89.da109()
                || da94.da104().da110() < 0.0
                || da94.da104().da110() > da34.da89.da111()
                ||
                da112
                (da94.da113() == da36.da114
                    && da63.da115.da116 > da34.da117);

        da118
        da81.da92(da94.da104());
        if (da97) {
          da83.da92(da94.da104());
        } else {
          da82.da92(da94.da104());
        }

        da119
        if (da97) {
          continue;
        }

        da120
        double da121 =
            da102.da122(da94.da123(), 2.0) / da94.da98();
        double da124 = da34.da125 * da121;
        double da126 = da34.da127 * da121;

        if (da57 < da34.da128.da56) {
          da124 *= da34.da128[da57];
          da126 *= da34.da128[da57];
        }

        if (da94.da113() == da36.da114) {
          da124 *= da34.da129;
          da126 *= da34.da130;
        }

        da131
        da49.da132(
            da94.da104().da133(),
            da94.da134(),
            da20.da135(da124, da124, da126));
      }

      da136
      da46.da137(
          da138 + da34.da55[da57] + da139,
          da80.da140(new da23[0]));
      da46.da137(
          da141 + da34.da55[da57] + da142,
          da81.da140(new da23[0]));
      da46.da137(
          da143 + da34.da55[da57] + da144,
          da82.da140(new da23[0]));
      da46.da137(
          da145 + da34.da55[da57] + da146,
          da83.da140(new da23[0]));

      da147
      da70.da148(da80);
      da71.da148(da81);
      da72.da148(da82);
      da73.da148(da83);
    }

    da149
    da46.da137(da150, da70.da140(new da23[0]));
    da46.da137(da151, da71.da140(new da23[0]));
    da46.da137(
        da152, da72.da140(new da23[0]));
    da46.da137(
        da153, da73.da140(new da23[0]));
  }

  @da61
  public void da154() {
    da155
  }

  @da156
  public interface da48 {
    void da132(
        da22 da157,
        double da158,
        da19<da26, da25> da159);
  }
}
