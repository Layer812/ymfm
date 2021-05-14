# ymfm

[ymfm](https://github.com/aaronsgiles/ymfm) is a collection of BSD-licensed Yamaha FM sound cores (OPM, OPN, OPL, and others), written by [Aaron Giles](https://aarongiles.com)

## Supported environments

This code should compile cleanly in any environment that has C++14 support.
It has been tested on gcc, clang, and Microsoft Visual C++ 2019.

## Supported chip families

Currently, support is present for the following chips (organized by header file):

* ymfm_opm.h:
	* YM2151 (OPM)
	* YM2164 (OPP)
* ymfm_opn.h:
	* YM2149 (SSG)
	* YM2203 (OPN)
	* YM2608 (OPNA)
	* YM2610 (OPNB)
	* YM2610B (OPNB2)
	* YM2612 (OPN2)
	* YM3438 (OPN2C)
	* YMF276 (OPN2L)
* ymfm_opl.h:
	* YM3526 (OPL)
	* Y8950 (MSX-Audio)
	* YM3812 (OPL2)
	* YMF262 (OPL3)
	* YMF278B (OPL4) -- partial (only the FM side)
	* YM2413 (OPLL)
	* YM2423 (OPLL-X)
	* YMF281 (OPLLP)
	* DS1001 (Konami 053982)
* ymfm_opq.h:
	* YM3806 (OPQ) -- preliminary
* ymfm_opz.h:
	* YM2414 (OPZ) -- preliminary

## General approach

Check out the [examples directory](https://github.com/aaronsgiles/ymfm/tree/main/examples) for some example usage patterns.
I'm not a big fan of makefiles for simple things, so instructions on how to compile each example are provided at the top.

### Clocking

The general philosophy of the emulators provided here is that they are clock-independent.
Much like the actual chips, you (the consumer) control the clock; the chips themselves have no idea what time it is.
They just tick forward each time you ask them to.

The way you move things along is via the `generate()` function, which ticks the internal system forward one or more samples, and writes out an array out chip-specific `output_data`.
But what, exactly, is a "sample", and how long is it?

This is where the external clock comes in.
Most of the Yamaha chips are externally clocked in the MHz range.
They then divide that clock by a factor (sometimes dynamically controllable), and then the internal operators are pipelined to further divide the clock.

For example, the YM2151 internally divides the clock by 2, and has 32 operators to iterate through.
Thus, for a nominal input lock of 3.58MHz, you end up at around a 55.9kHz sample rate.
Fortunately, all the chip implementations can compute this for you; just pass the raw external clock value to the `sample_rate()` method and it will hand you back the output sample rate you want.

Then call `generate()` that many times per second to output the results.

But what if I want to output at a "normal" rate, like 44.1kHz?
Sorry, you'll have to rate convert as needed.

### Reading and Writing

To read or write to the chips, you can call the `read()` and `write()` methods.
The offset provided corresponds to the addressing input lines in a (hopefully) logical way.

For reads, almost all chips have a status register, which you can read via `read_status()`.
Some chips have a data port that can be read via `read_data()`.
And chips with extended addressing may also have `read_status_hi()` and `read_data_hi()`.

For writes, almost all chips have an address register and a data register, and so you can reliably count on there being a `write_address()` and `write_data()` method as well.
If the chip supports extended addressing, it may also have `write_address_hi()` and `write_data_hi()`.

