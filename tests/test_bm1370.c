#include <assert.h>
#include <math.h>
#include <stdint.h>

#define BM1370_REF_FREQ 25.0
#define BM1370_FB_DEFAULT 0xA0
#define BM1370_REFD_DEFAULT 2
#define BM1370_PD1_DEFAULT 5
#define BM1370_PD2_DEFAULT 2

static float find_pll(float target, uint8_t *fb, uint8_t *refd,
                      uint8_t *pd1, uint8_t *pd2)
{
    float min_diff = 10.0f, diff, calc = 200.0f;
    uint8_t _fb = 0, _pd1 = 0, _pd2 = 0, _refd = 0;
    for (uint8_t r = 2; r > 0 && _fb == 0; r--) {
        for (uint8_t d1 = 7; d1 > 0 && _fb == 0; d1--) {
            for (uint8_t d2 = 7; d2 > 0 && _fb == 0; d2--) {
                if (d1 >= d2) {
                    int tmp = lroundf(((float)d1 * d2 * target * r) / BM1370_REF_FREQ);
                    if (tmp >= 0xa0 && tmp <= 0xef) {
                        calc = BM1370_REF_FREQ * tmp / ((float)r * d1 * d2);
                        diff = fabsf(target - calc);
                        if (diff < min_diff) {
                            _fb = tmp;
                            _pd1 = d1;
                            _pd2 = d2;
                            _refd = r;
                            min_diff = diff;
                        }
                    }
                }
            }
        }
    }
    if (_fb == 0) {
        _fb = BM1370_FB_DEFAULT;
        _refd = BM1370_REFD_DEFAULT;
        _pd1 = BM1370_PD1_DEFAULT;
        _pd2 = BM1370_PD2_DEFAULT;
        calc = BM1370_REF_FREQ * _fb / (_refd * _pd1 * _pd2);
    }
    if (fb) *fb = _fb;
    if (refd) *refd = _refd;
    if (pd1) *pd1 = _pd1;
    if (pd2) *pd2 = _pd2;
    return calc;
}

int main(void)
{
    uint8_t fb, refd, pd1, pd2;
    float out = find_pll(10.0f, &fb, &refd, &pd1, &pd2);
    assert(fb == BM1370_FB_DEFAULT);
    assert(refd == BM1370_REFD_DEFAULT);
    assert(pd1 == BM1370_PD1_DEFAULT);
    assert(pd2 == BM1370_PD2_DEFAULT);
    float expected = BM1370_REF_FREQ * BM1370_FB_DEFAULT /
                     (BM1370_REFD_DEFAULT * BM1370_PD1_DEFAULT * BM1370_PD2_DEFAULT);
    assert(fabsf(out - expected) < 1e-6);
    return 0;
}
