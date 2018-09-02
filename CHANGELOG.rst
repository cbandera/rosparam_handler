^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rosparam_handler
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.4 (2018-09-02)
------------------
* update doc
* fix `#57 <https://github.com/cbandera/rosparam_handler/issues/57>`_
  replace map/vec custom stream op by func
* update readme
* add toConfig
* Fix `#47 <https://github.com/cbandera/rosparam_handler/issues/47>`_
  Do not print error message while retrieving param with default value.
* Contributors: Jeremie Deray, artivis

0.1.3 (2018-03-23)
------------------
* Merge pull request `#50 <https://github.com/cbandera/rosparam_handler/issues/50>`_ from cbandera/develop
  Use industrial_ci & docker
* Merge pull request `#45 <https://github.com/cbandera/rosparam_handler/issues/45>`_ from cbandera/feature/travis_industrial_ci
  Change travis conf to use industrial_ci & docker
* Merge pull request `#48 <https://github.com/cbandera/rosparam_handler/issues/48>`_ from cbandera/develop
  Fix install rule
* Merge branch 'develop' into feature/travis_industrial_ci
* Merge pull request `#46 <https://github.com/cbandera/rosparam_handler/issues/46>`_ from plusone-robotics/plusone/master
  Fix CI Breaking on External Projects
* Merge pull request `#3 <https://github.com/cbandera/rosparam_handler/issues/3>`_ from geoffreychiou/gc_fix_ci
  Fixed CI Build Error Caused by rosparam_handler
* added slash at end of include dir
* Merge branch 'develop' into feature/travis_industrial_ci
* change travis conf to use industrial_ci & docker
  - using industrial_ci simplifies a lot the travis conf file
  - using Docker enables CI for kinetic & lunar
* Contributors: Claudio Bandera, Geoffrey Chiou, Jeremie Deray, geoffreychiou

0.1.2 (2018-02-07)
------------------
  Release 0.1.2
  fix `#34 <https://github.com/artivis/rosparam_handler/issues/34>`_. Append local path at the beginning of PYTHON_PATH
  Update maintainer
  Add test dynamic_reconfigure
* std::endl -> \n
  YamlGenerator closes `#30 <https://github.com/artivis/rosparam_handler/issues/30>`_
* Bugfixes
* Uncomment lines without default, to force user to set them
* Added documentation for generator script
* Reformatted python files
* ClangFormat files
* Simplified utility functions
* Fix for failing tests
* Added functionality for setting parameters on server.
* Added documentation to functions
* Moved common function to util header
* Removed debugging leftover
  Add Python support
* Display correct test names for each test
* Replaced errors with warnings
  For python, when min/max is corrected. The behaviour is now similar to cpp
* Made tests visible
  Replaced individual testcases with TestSuite
* Set default values on param server, add python tests for that
* Remove useless comment leftovers
  Bugfix/force clean builds
* Readded check for C++11 support.
* Force travis to do a clean build
* Removed verbose output during build
  Fix initialisation order in parameter class template
* Fixed merge error in template
  Remove useless character
  Set default values on parameter server if no value is set
* Updated the documentation
* Remove duplicate test
* Add unittests
* Name tests properly, add cmake dependency
* Throw more informative exception if parameters without default value are unset
* Add python unittests
  Feature/add continous integration
* Changed build flag to display develop branch
* Updated README contribution guide
* Added matrix job and removed C++14 check
* Added initial travis config
* Added first set of tests
* Added LICENSE file. Closes `#25 <https://github.com/artivis/rosparam_handler/issues/25>`_
* Updated README to include release information. Closes `#17 <https://github.com/artivis/rosparam_handler/issues/17>`_.
* Moved common functions to utilities.hpp. Fixes `#11 <https://github.com/artivis/rosparam_handler/issues/11>`_
* Handler is now throwing an exception upon failure.
  Before, it was calling std::exit(EXIT_FAILURE) but it was causing problems during testing.
* Add min/max check for map and vector to be consistent with C++
* Include python usage in documentation
* Set default values on parameter server if no value is set
* Fix initialisation order in parameter class template
* param handler now generates python parameter files too
* Remove useless character
* Contributors: Claudio Bandera, Fabian Poggenhans, Jan-Hendrik Pauls, Jeremie Deray, Niels Ole Salscheider, vincentrou

0.1.1 (2017-05-31)
------------------
* Initial release of rosparam_handler
* Contributors: Claudio Bandera, Fabian Poggenhans, Jeremie Deray, Matthias Füller, Nikolaus Demmel, Sascha Wirges, artivis
