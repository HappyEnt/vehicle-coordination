=============
Orcar Localization
=============

Main entry point for ROS2 node is defined in ``belief_propagation.cpp``.

The implementation is ROS agnostic and is implemented in the files

- ``particle-belief-propagation.c``
- ``util.c``
- ``debug.h``

The starting point for studying the belief propagation implementation is the  function ``iterate`` in ``particle-belief-propagation.c``.
Here we dispatch to the configured implementation.

Reading
-------

- ``Silverman, B. W. (2018). Density estimation for statistics and data analysis. Routledge.``
  Here you will find methods to emperically determine an ideal bandwidth value.

- ``De Freitas, N., & Gordon, N. J. (2001). Sequential Monte Carlo methods in practice (Vol. 1, No. 2). A. Doucet (Ed.). New York: Springer.``
  Here you will find the regularized particle filter, both post and pre versions.
  Also other interesting approaches like the RESAMPLE-MOVE variant, where after resampling a short
  MCMC simulation is run, can be found here.

- ``Doucet, A., & Johansen, A. M. (2009). A tutorial on particle filtering and smoothing: Fifteen years later. Handbook of nonlinear filtering, 12(656-704), 3.``
  The best introduction into particle filtering that I found.
