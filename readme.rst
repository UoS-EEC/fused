========================================================
Fused: Full-System Simulation of Energy-Driven Computers
========================================================

.. image:: https://travis-ci.com/UoS-EEC/fused.svg?branch=master
    :target: https://travis-ci.com/UoS-EEC/fused

Fused is a full-system simulator for modelling energy-driven computers.
To accurately model the interplay between energy-availability, power
consumption, and execution; Fused models energy and execution in a closed
feedback loop.

The power model is based on recording high-level events (memory accesses,
peripheral operations etc.) and states (modules on/off, peripheral operation
modes etc.), and computing the instantaneous power consumption at runtime.
Simultaneously, external circuitry such as e.g. energy storage, power supply
and power management is modelled. The power consumption, power supply, and
energy-availability, is then used to calculate the supply voltage, which in
turn is monitored by the modeled microcontroller.  In this way, we can model
an embedded system through power cycles.

Some key features of Fused include:

* Simulates power consumption and power supply in a closed feedback loop;
* Hosts a GDB server to interface with most software development environments;
* Enables debugging functionality across power cycles, and the ability to
  freeze and step through the dynamic power/energy state in lockstep with
  execution.
* Executes unmodified binaries to be deployed on real hardware;
* Allows modelling of complex external circuitry through SystemC-AMS.

Fused will be presented at the 2020 ISPASS conference.
Please cite the following paper when using Fused in your research.

    | "Fused: Closed-loop Performance and Energy Simulation of Embedded Systems",
    | Sivert T. Sliper, William Wang, Nikos Nikoleris, Alex S. Weddell, Geoff V. Merrett,
    | 2020 International Symposium on Performance Analysis of Systems and Software (ISPASS),
    | Boston, MA, USA, 2020.

The paper is available at `computer.org`_ , and the supporting dataset at
`DOI 10.5258/SOTON/D1200`_.


Directory structure
===================

+----------------------+-----------------------------------------------------+
| Directory            | Content                                             |
+======================+=====================================================+
| config               | Configuration files and power-model coefficients.   |
+----------------------+-----------------------------------------------------+
| doc                  | Documentation                                       |
+----------------------+-----------------------------------------------------+
| libs                 | Third-party libraries                               |
+----------------------+-----------------------------------------------------+
| mcu                  | Microcontroller models (CPU, memory, peripherals)   |
+----------------------+-----------------------------------------------------+
| ps                   | Power system/model                                  |
+----------------------+-----------------------------------------------------+
| scripts              | Utility scripts                                     |
+----------------------+-----------------------------------------------------+
| sw                   | Target software validation programs                 |
+----------------------+-----------------------------------------------------+
| sd                   | Serial devices, e.g. external chips communicating   |
|                      | over SPI.                                           |
+----------------------+-----------------------------------------------------+
| test                 | Hardware module tests                               |
+----------------------+-----------------------------------------------------+
| test-misc            | Miscellaneous integration tests (e.g trace-based)   |
+----------------------+-----------------------------------------------------+
| utilities            | Utilities                                           |
+----------------------+-----------------------------------------------------+

Build
=====

We've set up two alternatives for building and using Fused.
The easiest way to get started is to build and run Fused through Docker.
Docker can slow down compilation and execution though, so if you need more
performance, you can install Fused on your linux machine.

In either case, the first step is to clone the repository:

.. code-block:: bash

   $> git clone https://git.soton.ac.uk/energy-driven/fused
   $> cd fused

Continue the installation by following the instructions for `Using Docker`_
or `On linux (tested on Ubuntu 18.04)`_ below.


Using Docker
------------

First, make sure you have `Docker Engine`_ installed and running. Then launch
an interactive session on the prebuilt docker image for *Fused* development as
follows:

.. code-block:: bash

  # in fused/
  $> docker run -v ${PWD}:/opt/src -p 51000:51000 -it sivertism/fused-dev:latest /bin/bash

The first time you run this might take a while, as Docker downloads the
prebuilt image.

Within this session, you can build and run *Fused*. To build *Fused* run the following commands:

.. code-block:: bash

    #in docker session
    $> mkdir build && cd build
    $> cmake .. -GNinja -DTARGET_ARCH=msp430
    $> ninja

Note that ``TARGET_ARCH`` can be either ``cm0`` or ``msp430`` for the *Arm
Cortex-M0* or *MSP430FR5994* targets, respectively. Usage of *Fused* is
described in `Basic usage`_.

If you need to rebuild the docker image, e.g. to modify one of the
dependencies or add some tools to the image, run the following command:

.. code-block:: bash

    $> docker build -t local-fused-dev . --build-arg SSH_PRIVATE_KEY="$(cat ~/.ssh/id_rsa)"

On linux (tested on Ubuntu 18.04)
---------------------------------

First, install a few tools:

.. code-block:: bash

    $> sudo apt install libboost-dev build-essential g++ ninja-build git gdb

Then install a recent version of *CMake* (>= version 3.12):

.. code-block:: bash

    $> wget https://github.com/Kitware/CMake/releases/download/v3.15.4/cmake-3.15.4-Linux-x86_64.sh
    $> chmod a+x cmake*.sh
    $> sudo ./cmake*.sh --skip-license --prefix=/usr/local

Now download, build & install *Fused*'s dependencies and target toolchains,
using *CMake* (this may take a while):

.. code-block:: bash

    # in fused/
    $> mkdir build && cd build
    $> cmake .. -GNinja -DINSTALL_DEPENDENCIES=ON -DINSTALL_TARGET_TOOLCHAINS=ON
    $> ninja

By default, this installs to ``~/.local``, but you can providea different
install path with the ``EP_INSTALL_DIR`` variable, e.g.
``-DEP_INSTALL_DIR=${HOME}/fused-deps``.

To build target software, we need to set a few environment variables, add these
lines to your ``~/.bashrc`` (or your shell's equivalent):

.. code-block:: bash

    export ARM_GCC_ROOT=${HOME}/.local/arm-gcc
    export MSP430_GCC_ROOT=${HOME}/.local/msp430-gcc
    export MSP430_INC=${HOME}/.local/msp430-inc
    export PATH="${HOME}/.local/msp430-gcc/bin:${PATH}"
    export PATH="${HOME}/.local/arm-gcc/bin:${PATH}"

Now, to build *Fused*, disable ``INSTALL_DEPENDENCIES`` and select a target
platform (``cm0`` for *Arm Cortex-M0* or ``msp430`` for *MSP430FR5994*):

.. code-block:: bash

    # in fused/build
    $> cmake .. -GNinja -DINSTALL_DEPENDENCIES=OFF -DTARGET_ARCH=msp430
    $> ninja


Once the this has completed, there will be a ``fused`` executable in the
``build`` folder.

Build workloads / target software
=================================

`<sw/>`_ contains validation programs for Fused, along with a build system to
compile them. To compile the validation programs, ``cd`` into `<sw/>` make a
build folder and run CMake.

.. code-block:: bash

    $> cd sw && mkdir build && cd build
    $> cmake .. -GNinja -DTARGET_ARCH=msp430
    $> ninja

Make sure to completely clear the ``build`` directory if you build for one
target then switch to another.

Basic usage
===========

When Fused is launched, it will load configuration options from the
``config.yaml`` file located in the ``build`` directory, then optionally start
a GDB server, or load and execute a target binary.

Load and execute target binary
------------------------------

To load a hex-formatted binary file (hex-file), and immediately start
simulation, launch Fused with the ``-x`` option, followed by the path to the
hex-file:

.. code-block:: bash

   # in fused/build
   $> ./fused -x <path/to/program.hex>

Alternatively, set up `<config/config.yaml.in>`_ with ``GdbServer: False`` and
``ProgramHexFile: <path/to/program.hex>``, then rerun CMake/rebuild to update
``build/config.yaml``.

Fused will then run until one of the one of the exit conditions are hit (e.g.
``SimTimeLimit`` or when the target program stops simulation via
``SIMPLE_MONITOR``).

Hosting a GDB server with Fused
-------------------------------

Configure `<config/config.yaml.in>`_ to ``GdbServer: True``, rebuild/rerun
CMake, and launch fused without the ``-x`` option. Fused will then start a GDB
server and halt execution while waiting for a client connection.

To connect to the server, start your GDB client, and connect to the Fused  GDB
server, e.g. as follows:

.. code-block:: bash

   $> msp430-elf-gdb <path/to/program.hex>
   (gdb) tui enable
   (gdb) target remote :51000
   (gdb) load
   (gdb) break main
   (gdb) continue

License
==========

See `<LICENSE>`_

Contributions
==============
Contributions are accepted under the Apache 2.0 license. Only submit
contributions where you have authored all of the code.


.. _TBD: https://doi.org/xxxx
.. _DOI 10.5258/SOTON/D1200: http://dx.doi.org/10.5258/SOTON/D1200
.. _Docker Engine: https://docs.docker.com/install/
.. _computer.org: https://conferences.computer.org/ispass/2020/pdfs/ISPASS2020-6XKlsJVdID5VxIhCMcg4bY/479800a121/479800a121.pdf
