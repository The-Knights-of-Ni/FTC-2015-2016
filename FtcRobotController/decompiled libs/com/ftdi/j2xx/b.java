/*
 * Decompiled with CFR 0_101.
 */
package com.ftdi.j2xx;

final class b {
    static byte a(int n, int[] arrn, boolean bl) {
        int n2;
        int n3;
        int n4;
        byte by = b.b(n, arrn, bl);
        if (by == -1) {
            return -1;
        }
        if (by == 0) {
            arrn[0] = (arrn[0] & -49153) + 1;
        }
        if (n > (n3 = b.a(arrn[0], arrn[1], bl))) {
            n4 = n * 100 / n3 - 100;
            n2 = n % n3 * 100 % n3;
        } else {
            n4 = n3 * 100 / n - 100;
            n2 = n3 % n * 100 % n;
        }
        by = n4 < 3 ? 1 : (n4 == 3 && n2 == 0 ? 1 : 0);
        return by;
    }

    private static byte b(int n, int[] arrn, boolean bl) {
        int n2;
        int n3;
        byte by = 1;
        if (n == 0) {
            return -1;
        }
        if ((3000000 / n & -16384) > 0) {
            return -1;
        }
        arrn[0] = 3000000 / n;
        arrn[1] = 0;
        if (arrn[0] == 1 && (n2 = 3000000 % n * 100 / n) <= 3) {
            arrn[0] = 0;
        }
        if (arrn[0] == 0) {
            return by;
        }
        n2 = 3000000 % n * 100 / n;
        if (!bl) {
            if (n2 <= 6) {
                n3 = 0;
            } else if (n2 <= 18) {
                n3 = 49152;
            } else if (n2 <= 37) {
                n3 = 32768;
            } else if (n2 <= 75) {
                n3 = 16384;
            } else {
                n3 = 0;
                by = 0;
            }
        } else if (n2 <= 6) {
            n3 = 0;
        } else if (n2 <= 18) {
            n3 = 49152;
        } else if (n2 <= 31) {
            n3 = 32768;
        } else if (n2 <= 43) {
            n3 = 0;
            arrn[1] = 1;
        } else if (n2 <= 56) {
            n3 = 16384;
        } else if (n2 <= 68) {
            n3 = 16384;
            arrn[1] = 1;
        } else if (n2 <= 81) {
            n3 = 32768;
            arrn[1] = 1;
        } else if (n2 <= 93) {
            n3 = 49152;
            arrn[1] = 1;
        } else {
            n3 = 0;
            by = 0;
        }
        int[] arrn2 = arrn;
        arrn2[0] = arrn2[0] | n3;
        return by;
    }

    private static final int a(int n, int n2, boolean bl) {
        if (n == 0) {
            return 3000000;
        }
        int n3 = (n & -49153) * 100;
        if (!bl) {
            switch (n & 49152) {
                default: {
                    break;
                }
                case 49152: {
                    n3+=12;
                    break;
                }
                case 32768: {
                    n3+=25;
                    break;
                }
                case 16384: {
                    n3+=50;
                    break;
                }
            }
        } else if (n2 == 0) {
            switch (n & 49152) {
                default: {
                    break;
                }
                case 49152: {
                    n3+=12;
                    break;
                }
                case 32768: {
                    n3+=25;
                    break;
                }
                case 16384: {
                    n3+=50;
                    break;
                }
            }
        } else {
            switch (n & 49152) {
                default: {
                    break;
                }
                case 0: {
                    n3+=37;
                    break;
                }
                case 16384: {
                    n3+=62;
                    break;
                }
                case 32768: {
                    n3+=75;
                    break;
                }
                case 49152: {
                    n3+=87;
                }
            }
        }
        n3 = 300000000 / n3;
        return n3;
    }

    static final byte a(int n, int[] arrn) {
        int n2;
        int n3;
        int n4;
        byte by = b.b(n, arrn);
        if (by == -1) {
            return -1;
        }
        if (by == 0) {
            arrn[0] = (arrn[0] & -49153) + 1;
        }
        if (n > (n4 = b.a(arrn[0], arrn[1]))) {
            n2 = n * 100 / n4 - 100;
            n3 = n % n4 * 100 % n4;
        } else {
            n2 = n4 * 100 / n - 100;
            n3 = n4 % n * 100 % n;
        }
        by = n2 < 3 ? 1 : (n2 == 3 && n3 == 0 ? 1 : 0);
        return by;
    }

    private static byte b(int n, int[] arrn) {
        int n2;
        int n3;
        byte by = 1;
        if (n == 0) {
            return -1;
        }
        if ((12000000 / n & -16384) > 0) {
            return -1;
        }
        arrn[1] = 2;
        if (n >= 11640000 && n <= 12360000) {
            arrn[0] = 0;
            return by;
        }
        if (n >= 7760000 && n <= 8240000) {
            arrn[0] = 1;
            return by;
        }
        arrn[0] = 12000000 / n;
        arrn[1] = 2;
        if (arrn[0] == 1 && (n3 = 12000000 % n * 100 / n) <= 3) {
            arrn[0] = 0;
        }
        if (arrn[0] == 0) {
            return by;
        }
        n3 = 12000000 % n * 100 / n;
        if (n3 <= 6) {
            n2 = 0;
        } else if (n3 <= 18) {
            n2 = 49152;
        } else if (n3 <= 31) {
            n2 = 32768;
        } else if (n3 <= 43) {
            n2 = 0;
            int[] arrn2 = arrn;
            arrn2[1] = arrn2[1] | 1;
        } else if (n3 <= 56) {
            n2 = 16384;
        } else if (n3 <= 68) {
            n2 = 16384;
            int[] arrn3 = arrn;
            arrn3[1] = arrn3[1] | 1;
        } else if (n3 <= 81) {
            n2 = 32768;
            int[] arrn4 = arrn;
            arrn4[1] = arrn4[1] | 1;
        } else if (n3 <= 93) {
            n2 = 49152;
            int[] arrn5 = arrn;
            arrn5[1] = arrn5[1] | 1;
        } else {
            n2 = 0;
            by = 0;
        }
        int[] arrn6 = arrn;
        arrn6[0] = arrn6[0] | n2;
        return by;
    }

    private static int a(int n, int n2) {
        if (n == 0) {
            return 12000000;
        }
        if (n == 1) {
            return 8000000;
        }
        int n3 = (n & -49153) * 100;
        if ((n2&=65533) == 0) {
            switch (n & 49152) {
                default: {
                    break;
                }
                case 49152: {
                    n3+=12;
                    break;
                }
                case 32768: {
                    n3+=25;
                    break;
                }
                case 16384: {
                    n3+=50;
                    break;
                }
            }
        } else {
            switch (n & 49152) {
                default: {
                    break;
                }
                case 0: {
                    n3+=37;
                    break;
                }
                case 16384: {
                    n3+=62;
                    break;
                }
                case 32768: {
                    n3+=75;
                    break;
                }
                case 49152: {
                    n3+=87;
                }
            }
        }
        n3 = 1200000000 / n3;
        return n3;
    }
}

