#
# This file is part of Mapnik (c++ mapping toolkit)
#
# Copyright (C) 2009 Artem Pavlenko, Dane Springmeyer
#
# Mapnik is free software; you can redistribute it and/or
# modify it under the terms of the GNU Lesser General Public
# License as published by the Free Software Foundation; either
# version 2.1 of the License, or (at your option) any later version.
#
# This library is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# Lesser General Public License for more details.
#
# You should have received a copy of the GNU Lesser General Public
# License along with this library; if not, write to the Free Software
# Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
#
# 

import os
from copy import copy

Import ('env')

source = Split(
    """
    debug.cpp
    """
    )
    


demo_env = env.Clone()


demo_env['CXXFLAGS'] = copy(env['LIBMAPNIK_CXXFLAGS'])
demo_env.Append(CPPDEFINES = env['LIBMAPNIK_DEFINES'])

if env['HAS_CAIRO']:
    demo_env.PrependUnique(CPPPATH=env['CAIRO_CPPPATHS'])
    demo_env.Append(CPPDEFINES = '-DHAVE_CAIRO')

libraries = ['mapnik']
libraries.extend(copy(env['LIBMAPNIK_LIBS']))
debug = demo_env.Program('debug', source, LIBS=libraries)

Depends(debug, env.subst('../../src/%s' % env['MAPNIK_LIB_NAME']))

# build locally if installing
if 'install' in COMMAND_LINE_TARGETS:
    env.Alias('install',debug)
