"""
WAF Tool to select the correct toolchain based on the target archtecture.

This tool must be loaded before compiler tools. Use the environment variable
TOOLCHAIN to define the toolchain prefix.

Example::

    def configure(cfg):
        cfg.env.TOOLCHAIN = 'arm-linux-gnueabihf'
        cfg.load('toolchain')
        cfg.load('cxx_compiler')
"""

from waflib import Utils
import os

suffixes = dict(
    CXX='g++',
    CC='gcc',
    AS='gcc',
    AR='ar',
    LD='g++',
    GDB='gdb',
    OBJCOPY='objcopy',
)

def configure(cfg):
    if not cfg.env.TOOLCHAIN:
        cfg.env.TOOLCHAIN = 'native'
        prefix = ''
    else:
        cfg.env.TOOLCHAIN = Utils.to_list(cfg.env.TOOLCHAIN)[0]
        cfg.msg('Using toolchain prefix', cfg.env.TOOLCHAIN)
        prefix = cfg.env.TOOLCHAIN + '-'

    if 'clang' in os.environ['CC'] or 'clang' in cfg.options.check_c_compiler:
        if cfg.env.TOOLCHAIN != 'native':
            cfg.env.CXXFLAGS += [
                '--target=' + cfg.env.TOOLCHAIN, 
                '--sysroot=/home/travis/opt/tools-master/arm-bcm2708/gcc-linaro-arm-linux-gnueabihf-raspbian-x64/arm-linux-gnueabihf/'
            ]	
    else:
        for k in suffixes:
            cfg.env.append_value(k, prefix + suffixes[k])
