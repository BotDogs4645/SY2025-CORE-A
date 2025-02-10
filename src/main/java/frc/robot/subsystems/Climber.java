package da1.da2.da3;

import da4.da5.da6.da7.da8;
import da4.da5.da6.da9.da10;
import da4.da5.da6.da9.da11;
import da4.da5.da6.da12.da13;
import da4.da5.da6.da14.da15;
import da4.da5.da6.da14.da16;

import da17.da18.da19.da20.da21;
import da17.da18.da19.da22.da23.da24;
import da17.da18.da19.da20.da25.da26;

public class da27 extends da24 {
    private final da8 da28;
    private final da21 da29;

    private static final int da30 = 10;
    private static final int da31 = 0;

    private static final double da32 = 200.0;
    private static final double da33 = 4096.0;
    private static final double da34 = da33 * da32;
    private static final double da35 = da34 / 360.0;

    da36
    private static final double da37 = 50.0;
    private static final double da38 = 50.0;
    private static final double da39 = 500.0;

    private static final double da40 = 170.0;
    private static final double da41 = -50.0;

    private double da42 = 0.0;

    private final da13 da43 = new da13(0).da44(0);


    public da27() {
        da28 = new da8(da30);
        da29 = new da21(da31);

        da10 da45 = new da10();


        da46
        da45.da47.da48(da15.da49);
        da45.da47.da50(da16.da51);

        da11 da52 = new da11();
        da52.da53 = da37;
        da52.da54 = da38;
        da52.da55 = da39;
        da45.da56 = da52;

        da28.da57().da58(da45);

        da42 = da29.da59();
        da42 = da42 * 360 * da32;

        da28.da60(da42);

        da26.da61(da62, da42);
    }

    @da63
    public void da64() {
        da26.da61(da65, da66());
        da26.da61(da67, da29.da59());
        da26.da61(da68, da28.da69().da70());
    }

    public void da71(double da72) {
        double da73 = da74.da75(da40, da74.da76(da41, da72)); da77
        double da78 = da73 / 360.0 * da32; da79

        da43.da60(da78);
        da28.da80(da43);

        da26.da61(da81, da73);
    }

    public double da66() {
        da82
        return (da29.da83() ) * 360.0;
    }

    public void da84() {
        da28.da84();
    }

    public void da85(double da86) {
        da28.da87(da86);
    }
}