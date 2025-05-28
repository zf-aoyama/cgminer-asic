# BM1370 Support

This repository adds experimental support for the Bitmain BM1370 ASIC.

## Building

```
./autogen.sh
./configure --enable-bitaxe
make -j"$(nproc)"
```

## Notes

* Frequency is set via dynamic PLL control. If the optimal parameters
  cannot be determined the driver falls back to a safe 200&nbsp;MHz
  configuration using default PLL values.
* Version rolling is enabled through `version_mask` by default.
* Only single-chip configurations are currently supported.
