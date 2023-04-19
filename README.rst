Introduction
============


.. image:: https://readthedocs.org/projects/circuitpython-laser-egismos/badge/?version=latest
    :target: https://circuitpython-laser-egismos.readthedocs.io/
    :alt: Documentation Status



.. image:: https://img.shields.io/discord/327254708534116352.svg
    :target: https://adafru.it/discord
    :alt: Discord


.. image:: https://github.com/furbrain/CircuitPython_laser_egismos/workflows/Build%20CI/badge.svg
    :target: https://github.com/furbrain/CircuitPython_laser_egismos/actions
    :alt: Build Status


.. image:: https://img.shields.io/badge/code%20style-black-000000.svg
    :target: https://github.com/psf/black
    :alt: Code Style: Black

Device driver for the egismos series of lasers, available at
https://www.egismos.com/laser-measuring-optoelectronics-module


Dependencies
=============
This driver depends on:

* `Adafruit CircuitPython <https://github.com/adafruit/circuitpython>`_

Please ensure all dependencies are available on the CircuitPython filesystem.
This is easily achieved by downloading
`the Adafruit library and driver bundle <https://circuitpython.org/libraries>`_
or individual libraries can be installed using
`circup <https://github.com/adafruit/circup>`_.

Installing from PyPI
=====================

On supported GNU/Linux systems like the Raspberry Pi, you can install the driver locally `from
PyPI <https://pypi.org/project/circuitpython-laser-egismos/>`_.
To install for current user:

.. code-block:: shell

    pip3 install circuitpython-laser-egismos

To install system-wide (this may be required in some cases):

.. code-block:: shell

    sudo pip3 install circuitpython-laser-egismos

To install in a virtual environment in your current project:

.. code-block:: shell

    mkdir project-name && cd project-name
    python3 -m venv .venv
    source .env/bin/activate
    pip3 install circuitpython-laser-egismos

Installing to a Connected CircuitPython Device with Circup
==========================================================

Make sure that you have ``circup`` installed in your Python environment.
Install it with the following command if necessary:

.. code-block:: shell

    pip3 install circup

With ``circup`` installed and your CircuitPython device connected use the
following command to install:

.. code-block:: shell

    circup install laser_egismos

Or the following command to update an existing version:

.. code-block:: shell

    circup update

Usage Example
=============

.. code-block:: python

    import time

    import board
    import busio
    import digitalio

    from laser_egismos import Laser

    laser_power = digitalio.DigitalInOut(board.D10)
    laser_power.switch_to_output(True)


    uart = busio.UART(board.D8, board.D9, baudrate=9600)
    laser = Laser(uart)
    laser.set_buzzer(False)
    laser.set_laser(True)
    time.sleep(3)
    laser.set_laser_(False)
    time.sleep(0.1)
    print(f"Distance is {laser.distance}cm")


Documentation
=============
API documentation for this library can be found on `Read the Docs
<https://circuitpython-laser-egismos.readthedocs.io/>`_.

For information on building library documentation, please check out `this guide
<https://learn.adafruit.com/creating-and-sharing-a-circuitpython-library/sharing-our-docs-on-readthedocs#sphinx-5-1>`_.

Contributing
============

Contributions are welcome! Please read our `Code of Conduct
<https://github.com/furbrain/CircuitPython_laser_egismos/blob/HEAD/CODE_OF_CONDUCT.md>`_
before contributing to help this project stay welcoming.
