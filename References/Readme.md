To see the assembly output: add `arm-none-eabi-objdump -D "${BuildArtifactFileBaseName}.elf" > "${BuildArtifactFileBaseName}.lst"`
to Project > Properties > C/C++ Build > Settings > Build Steps (tab) > Post-build steps.
[Source](https://www.openstm32.org/forumthread2152)

It is recommended to set optimization in Release builds to -O3 rather than the default -Osize.
Change this in Project > Properties > C/C++ Build > Settings > Tool Settings (tab) > MCU GCC Compiler >
Optimization > Optimization Level. You may need to click in and out of Settings to get the right tab to show.


## Recommended Compiler Warnings
I recommend turning these on in every project you make. Go to Project > Properties > C/C++ Build > Settings >
MCU GCC Compiler > Miscllaneous. Don't forget to add them for Debug and Release.
* `-Wshadow` - Warns if you declare a variable with the same name as a global variable.
* `-Winline` - Warns if a function specified as inline can't be inlined.
* `-Wvla` - Warns if a variable length array (vla) is written.
* `-Wstrict-prototypes` - This requires complete prototypes / forward declarations of functions. In C, a
	function declaration like `void func()` is actaully a function that can take any number of arguments,
	not no arguments. For a no argument function, write `void func(void)`.
* `-Wlogical-op` - Warns if an express is redundant in a logical operation.
* `-Wshift-negative-value` - Warns when left shifting a negative value, which can interfere with the sign bit.
* `-Wtype-limits` - Warns if a comparison to a type is always true or always false.
* `-Wmissing-parameter-type` - Warns if you don't write a parameter type (ex: int func(arg)) which defaults to int.
* `-Wmissing-prototypes` - Warns if you don't have a prototype for a function ahead of it's definition.
* `-Wmissing-field-initializers` - Warns if you initialize a struct and omit initializing a field, possibly because it was forgotten.
* `-Wimplicit-fallthrough=2` - Warns if a `case` statement in a switch does not have `break` before the following `case`.
    Writing "fallthrough" or variations (see GCC warnings page) signals that the omission is intentional and no warning is emitted.
* `-Wcast-qual` - Warns if you cast away `const`-ness of a variable/expression.
* `-Wignored-qualifier` - Warns if there's a qualifier in a function return type (`const` usually) that does nothing. Possibly misplaced and hiding a bug.
* `-Wpointer-arith` - Warns if you take the `sizeof` function or void. Bad practice and hopefully unintentional.
* `-Wunsafe-loop-optimizations` - Warns if loop optimizations can't be done because the loop condition is weird.
* `-Wduplicated-branches` - Warns if two branches in an `if` statement do the same thing.
* `-Wduplicated-cond` - Warns if two conditions in an if-elseif chain have the same condition.



The below warnings will produce many warnings in the STM generated code, but I recommend you add them to
any file you write.

* `#pragma GCC diagnostic warning "-Wunused-macros"` - Warns if a macro is not used.
* `#pragma GCC diagnostic warning "-Wunused-parameter"` - Warns if a function parameter is not used.
* `#pragma GCC diagnostic warning "-Wsign-compare"` - Warns about comparing a signed to an unsigned value.
* `#pragma GCC diagnostic warning "-Wconversion"` - Warns about implicit conversions between types.
* `#pragma GCC diagnostic warning "-Wredundant-decls"` - Warns about a variable that is redeclared.
* `#pragma GCC diagnostic warning "-Wswitch-default"` - Warns if a `switch` has no `default` case.
* `#pragma GCC diagnostic warning "-Wswitch-enum"` - Warns if the expression in the `switch` is an `enum` and one enum constant is missing a `case`.


Enabling strict prototypes and missing prototypes will cause warnings in `syscalls.c`. Replace `errno` with the following
code and insert the missing `void` to fix these warnings.
```C
#pragma GCC diagnostic ignored "-Wmissing-prototypes"
#pragma GCC diagnostic ignored "-Wstrict-prototypes"
extern int errno;
#pragma GCC diagnostic warning "-Wstrict-prototypes"
```
And add `void *_sbrk(ptrdiff_t inc);` to line 32 in `sysmem.c`.


**References:**
* [GCC Warning Options](https://gcc.gnu.org/onlinedocs/gcc/Warning-Options.html#Warning-Options)
* [GCC Pragmas for changing warnings](https://gcc.gnu.org/onlinedocs/gcc/Diagnostic-Pragmas.html)


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
