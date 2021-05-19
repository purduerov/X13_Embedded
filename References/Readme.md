To see the assembly output: add `arm-none-eabi-objdump -D "${BuildArtifactFileBaseName}.elf" > "${BuildArtifactFileBaseName}.lst"`
to Project > Properties > C/C++ Build > Settings > Build Steps (tab) > Post-build steps.
[Source](https://www.openstm32.org/forumthread2152)

It is recommended to set optimization in Release builds to -O3 rather than the default -Osize.
Change this in Project > Properties > C/C++ Build > Settings > Tool Settings (tab) > MCU GCC Compiler > Optimization > Optimization Level.
You may need to click in and out of Setting to get the right tab to show.

## Useful macros:

### Compiler time (aka static) assert
```C
#define ASSERT_CONCAT_(a, b) a##b
#define ASSERT_CONCAT(a, b) ASSERT_CONCAT_(a, b)
#define compile_assert(e) enum { ASSERT_CONCAT(assert_line_, __LINE__) = 1/(!!(e)) }
// From: http://www.pixelbeat.org/programming/gcc/static_assert.html
```
Usage:
```C
compile_assert(sizeof(my_struct_t) == 4);
```


### Number of elements in an array
```C
#define N_ELEMENTS(ARR) (sizeof(ARR) / sizeof(ARR[0]))
```
Usage
```C
for(uint32_t i = 0; i < N_ELEMENTS(buffer); ++i) {
```


### Mask of lowest n bits
```C
#define MASK_OF(N_BITS) (((N_BITS) << 1) - 1)
```
Usage
```C
uint8_t lowestByte = data & MASK_OF(8);  // Macro will return 0xFF
```
