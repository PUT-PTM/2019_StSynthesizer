# 2019_StSynthesizer

StSynthesizer is a DIY musical instrument made using the STM32F407 Discovery Board. It's using mathematical functions to generate different kind of waves. For example sine wave, square wave, triangle wave etc.

## Table of Contents
* [About the Project](#about-the-project)
  * [Built With](#built-with)
* [Getting Started](#getting-started)
* [Authors](#authors)
* [License](#license)

## About The Project

StSynthesizer is a project for our university classes. 
This projects uses STM32F407 Discovery Board which is easy to program and use.
The main goal of this project was to create something that would be able to play various sounds.

For now we've came with this features:
* The StSynthesizer comes with 12 playable buttons which makes it possible to play one octave 
* You can play 5 different sounds, which are:
  * Sine wave
  * Square wave
  * Triangle wave
  * Saw wave
  * Noise
* You're able to change the current octave, up to 8th octave
* Change volume

There is also a lot of work to be done:
* Add more sounds (with addative synthesis you can theoretically achieve any sound possible)
* Add attack and release time (when you press and release button)
* Add sequencer
* Fix bugs

### Built With
This project was built using:
* [STM32F407VG](https://www.st.com/content/st_com/en/products/microcontrollers-microprocessors/stm32-32-bit-arm-cortex-mcus/stm32-high-performance-mcus/stm32f4-series/stm32f407-417/stm32f407vg.html)
* [STM32CubeMX](https://www.st.com/en/development-tools/stm32cubemx.html)
* [SW4STM32 IDE](https://www.st.com/en/development-tools/sw4stm32.html)
* [16 tact switch buttons 12mm x 12mm]
* [Potentiometer]

## Getting Started

If you'd like to use this project locally you can download it and, open it in SW4STM32 IDE and upload the code to your STM32F407 board.
You can check all pins in STM32CubeMX. 

## Authors

* **Jakub Kusiowski** - [JKusio](https://github.com/JKusio)
* **Nikodem Janaszak** - [NikodemJanaszak](https://github.com/NikodemJanaszak)



## License

Distributed under the MIT License. See `LICENSE` for more information.
