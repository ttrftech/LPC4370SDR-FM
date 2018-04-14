LPC4370 SDR - Standalone SDR experiment using multicore MCU - FM version
==========================================================

<div align="center">
<img src="/doc/LPC-Link2-FM.jpg" width="480px">
</div>

# About

MCU based Standalone SDR implementation with LCD using multicore and integrated high speed ADC.

This project is derived from article of Interface magazine (Aug 2016) from CQ publishing.


# Prerequisite

* Hardware
   * LPC-Link2 
   * I2S Codec & LCD daughterboard
* IDE
   * LPCXpresso 

# How to build

Launch lpcxpresso, activate free edition.

## Import libraries

Import projects to workspace from Quickstart pane.

* CMSIS_DSPLIB_CM0
* CMSIS_DSPLIB_CM4
* CMSIS_LPC43xx_DriverLib 
* CMSIS_LPC43xx_DriverLib-M0

## Import projects

Download .zip, and import it to workspace.

## Build projects

Build each project.

* DisplayM0APP  (M0 App core)
* FMReceiverMC (M4)
