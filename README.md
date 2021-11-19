# WiredHoo

Project to add **wired** connectivity to a Wahoo Kick by emulating the ANT wireless protocol within an emulated ANT USB stick; the host believes it is communicating to an ANT+ wireless device.

Fed up with wireless connectivity issues, I decided to come up with a solution. The ergo trainer and inertial measurement stems from the work done by [Budget Trainer Build](https://budgettrainerbuild.wordpress.com/)

Broader scope for this project is to become an open-source firmware alternative for inertial trainers.

The project is very much a work in progress and not top of my priority - I'm open to collaboration. See the GitHub discussions board for upto date progress.

## TODO - See GitHub discussions

 - [x] Create ANT USB device.
- [x] Emulate ANT control.
- [x] Emulate ANT+ power meter and fitness control profiles.
- [x] CI tests for the above - more as develop.
- [x] Light sensor speed reading: TIM counter input capture, I’ve got most of this just need to test.
- [x] Moment of inertia based power measurement: based on work done by Budget trainer build
- [ ] Back EMF resistance control: PID controller based on measured power to setpoint error.
- [ ] Spin-down routine: fit curve of energy loss with rps.
- [x] ADC sampling: vsense, emf sense - the ADC is initialised but thinking DMA circular buffer.
- [ ] LED routines 💄: basic but also thinking WS2812B for power level indication.

# Build

## Requirements

* `arm-none-eabi-gcc` (version 10 used) and a SWD flashing tool (Black Magic Probe, STLINK or similar) for development.
* `make upload` assumes a [Black Magic Probe](https://1bitsquared.com/products/black-magic-probe) is being used with [Tagconnect TC2030 CTX](https://www.tag-connect.com/product/tc2030-ctx-nl-6-pin-no-legs-cable-with-10-pin-micro-connector-for-cortex-processors).
* An STM32F4 dev board: I'm using the Feather STM32F4. Will probably make custom PCB at some point to replace Kickr board.
* _STM32CubeMX if changing chip setup_

```
make # compile with debug flags
make upload # upload to board via SWD
make attach # attach for GDB debug
```

# Porting

I started this project around the Wahoo Kick V4. It should be possible to port this to any moment of inertia based trainer (or otherwise for that matter):

* trainer.h has many defines specific to the trainer: inertia of system, dimensions etc. If you want to develop for another trainer, I suggest we break these into trainer specific header files.
* The timer counter capture (tim.c/.h) setup might require changing depending on the expected frequency of the flywheel speed measurement.
* Inevitably other bits to be added here.
