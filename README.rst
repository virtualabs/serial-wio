Serial-Wio
==========

Turn your WioTerminal into an automagic UART terminal !

Features
--------

* Automatic baudrate/parity detection
* On-screen UART log
* USB UART bridge

Case printing and assembly
--------------------------

A custom case has been designed to fit the WioTerminal, it allows normal multimeter probes
to be used and protects the electronics.

The case is made of 3 independent parts that may be 3d printed with any decent 3d printer:

* a bottom part that holds 6 M3 hex screws
* a middle part that holds 2 M3 hex nuts
* a top part

Extra materials required:

* 2x 10mm M3 screws
* 4x 15mm M3 screws
* 2x M3 nuts

Printing the case
~~~~~~~~~~~~~~~~~

Top part must be printed upside down to get best results, without supports. Bottom part must be
printed with supports to get the screws' heads holes correctly printed.

Assembling the case
~~~~~~~~~~~~~~~~~~~

First, insert 2 10mm M3 screws in the bottom part as shown below:

.. image:: img/assemblage-step1.png
  :alt: Assembling bottom part

Then, place the custom PCB inside the bottom part, as shown below:

.. image:: img/assemblage-step2.png
  :alt: Insert Serial-Wio assembled PCB

Place on top of this the intermediate lid and add two M3 nuts. Tighten them just enough to hold the lid
in place. 

.. image:: img/assemblage-step3.png
  :alt: Place and secure intermediate lid in place

Last, place the top lid and screw 4x 15mm M3 screws to maintain it in place, as shown below:

.. image:: img/assemblage-step4.png
  :alt: Fix the top lid

Custom PCB
----------

We designed a custom PCB that extends WioTerminal capabilities designed with KiCAD v5.
Design files are available in `PCB_design_files` folder, including 3D models and custom
fingerprints.

Gerber files are also provided in the `PCB_design_files/gerber` directory, as well as the
`kicad_pcb` file if you want to order it on OSHPark. 


