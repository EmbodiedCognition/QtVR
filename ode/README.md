# -*- mode: markdown -*-

This file contains notes on how to build ODE for use in our skeleton modeling
code.

First, check out the version of ODE that corresponds to the patch revision
you're using. My recommendation is to use the latest patch:

    svn checkout -r 1939 http://svn.code.sf.net/p/opende/code/trunk opende

But if for some reason you need to use an older version, the ODE SVN repository
changed URLs around r1904, so you might need to use something like this:

    svn checkout -r 1916 http://opende.svn.sf.net/svnroot/opende/trunk opende

Then, apply the patch to your checked-out copy of ODE:

    cd opende
    patch -p0 < ../QtVR/ode/ode-r1939.patch

Generate the configuration scripts, configure, make, and make install:

    ./bootstrap
    ./configure --enable-double-precision --enable-shared
    make -j4
    make install

Assuming you have permissions to install to /usr/local. If not, either sudo make
install on that last line, or add --prefix /path/to/my/workspace to the
configure invocation.
