package da1.da2.da3;

import static da4.da5.da6.da7.da8.da9;
import static da4.da5.da6.da7.da8.da10;

import da11.da12.da13;
import da11.da14.da15;
import da11.da16.da17;
import da18.da19.da20.da21;
import da18.da19.da20.da22;
import da18.da19.da20.da23.da24;
import da18.da19.da20.da23.da25;
import da18.da19.da20.da23.da26;
import da4.da5.da6.da27.da28;
import da4.da5.da6.da27.da29.da30;
import da4.da5.da6.da27.da29.da31;
import da4.da5.da6.da27.da32.da33;
import da4.da5.da6.da27.da34.da35;
import da4.da5.da6.da27.da34.da36;
import da4.da5.da6.da37.da38;
import da4.da5.da6.da37.da38.da39;
import da4.da5.da6.da37.da40;
import da4.da5.da6.da37.da41;
import da4.da5.da6.da42.da43.da44;
import da4.da5.da6.da42.da43.da45;
import da4.da5.da6.da42.da43.da46.da47;
import da1.da2.da48.da49;
import da1.da2.da50.da51.da52;
import da53.da54.da55.da56;
import da57.da58.da59.da60;

da61
public class da62 extends da52 implements da45 {
  private static final double da63 = 0.005; da64
  private da40 da65 = null;
  private double da66;

  da67
  private static final da31 da68 = da31.da69;
  da70
  private static final da31 da71 = da31.da72;
  da73
    if (!da74 || da38.da75()) {
      da38.da76()
          .da77(
              da78 -> {
                da79(
                    da78 == da39.da80
                        ? da71
                        : da68);
                da74 = true;
              });
    }
  }

  private void da81() {
    da66 = da22.da82();

    da83
    da65 =
        new da40(
            () -> {
              final double da84 = da22.da82();
              double da85 = da84 - da66;
              da66 = da84;

              da86
              da87(da85, da41.da88());
            });
    da65.da89(da63);
  }

  da90
  public void da91(da17 da92) {
    da60.da93(da94, da92.da95());

    da96
    da30 da97 = da98().da99;

    da100
    da33 da101 =
        new da33(
            da92.da102 + da49.da103.da104(da97.da105(), da92.da106),
            da92.da107 + da49.da108.da104(da97.da109(), da92.da110),
            da92.da111
                + da49.da112.da104(
                    da97.da113().da114(), da92.da115));

    da116
    da117(
        da118
            .da119(da101)
            .da120(da92.da121())
            .da122(da92.da123()));
  }

  public da15 da124(da13<da17> da125) {
    return new da15(
        () -> da98().da99, this::da126, this::da91, true, this, da125);
  }
}
