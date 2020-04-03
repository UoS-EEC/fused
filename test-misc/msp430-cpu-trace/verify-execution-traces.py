#!/usr/bin/env python3
#
# Copyright (c) 2019-2020, University of Southampton and Contributors.
# All rights reserved.
#
# SPDX-License-Identifier: Apache-2.0
#

import os, glob, re, code, sys
import subprocess as sp
from pathlib import Path

# Paths
odir = '/tmp/traces/'  # Simulated traces
idir = '/tmp/measured-traces'  # Measured traces
bindir = str(Path('./binaries'))
fuseddir = os.path.join(Path.home(), 'git', 'fused', 'build')

elfs = [
    'aes-FFF-WS0', 'matmul-FFF-WS0', 'matmul-tiled-FFF-WS0', 'matmul-FSS-WS0',
    'matmul-tiled-FSS-WS0', 'aes-FSS-WS0', 'newlib-sqrt-FSS-WS0',
    'nettle-sha256-FSS-WS0'
]

npts = int(10000)


def parseTrace(content):
    blocks = re.split(r'\n0x[0-9a-f]+ in .+', content)[1:]
    blocks = [b for b in blocks if "Loading" not in b]
    regs = [
        'pc', 'sp', 'sr', 'cg', 'r4', 'r5', 'r6', 'r7', 'r8', 'r9', 'r10',
        'r11', 'r12', 'r13', 'r14', 'r15'
    ]
    state = [{} for i in range(len(blocks))]
    for i, b in enumerate(blocks):
        for r in regs:
            regex = r'{}\s+(0x[0-9a-f]+)\s'.format(r)
            state[i][r] = int(re.search(regex, b).group(1), 16)
    return state


def compareValues(d1, d2):
    for k in d1.keys():
        if (d1[k] != d2[k]):
            print('{}: 0x{:04x}(sim), 0x{:04x}(meas)'.format(k, d1[k], d2[k]))
            return False
    return True


def filterConsecutive(arr):
    return [
        arr[i] for i in range(len(arr) - 1) if arr[i]['pc'] != arr[i + 1]['pc']
    ]


def emitReproduceScript(bp, path, elf):
    print('Emitting reproduce script at {}'.format(path))
    for mode in ['sim', 'meas']:
        port = 51000 if mode == "sim" else 55000
        with open(os.path.join(path, 'reproduce-' + mode + '.gdb'),
                  'w+') as of:
            of.write('''
          set pagination off
          set verbose off
          layout asm
          layout regs
          file {}.elf
          b *0x{:04x}
          target remote :{:d}
          load
          set $r4 = 0
          set $r5 = 0
          set $r6 = 0
          set $r7 = 0
          set $r8 = 0
          set $r9 = 0
          set $r10 = 0
          set $r11 = 0
          set $r12 = 0
          set $r13 = 0
          set $r14 = 0
          set $r15 = 0
          c
          '''.format(os.path.join(path, elf), bp, port))


def emitTraceScripts(path, elf, npts):
    with open('trace.gdb', 'r') as infile:
        original = infile.read()
    original = re.sub('while \(\$i \< \d+\)', 'while ($i < {:d})'.format(npts),
                      original)
    original = re.sub('#file', 'file {}.elf'.format(os.path.join(path, elf)),
                      original)
    with open(os.path.join(path, 'trace-meas.gdb'), 'w+') as of:
        of.write(re.sub(':\d+', ':55000', original))
    with open(os.path.join(path, 'trace-sim.gdb'), 'w+') as of:
        of.write(re.sub(':\d+', ':51000', original))


def runTrace(path, dev):
    if (dev == 'sim'):
        # Start Fused
        fusedRes = sp.Popen(
            [fuseddir + '/fused', '-C', fuseddir + '/config.yaml'],
            stdout=sp.PIPE,
            stderr=sp.PIPE)
    cmd = [
        'msp430-elf-gdb', '-x',
        os.path.join(path, 'trace-{}.gdb'.format(dev))
    ]
    res = sp.run(cmd, stdout=sp.PIPE, universal_newlines=True)
    return res.stdout


def main():
    # Extract measured traces
    if not Path(idir).exists():
        os.makedirs(idir)
    cmd = [
        'tar', '-xjf', 'meas-cpu-traces.tar.bz2', '--directory', idir,
        '--strip-components', '1'
    ]
    res = sp.run(cmd, stdout=sp.PIPE, universal_newlines=True)

    anyFailed = False
    for elf in elfs:
        # Set up files
        testdir = os.path.join(odir, elf)
        if not os.path.exists(testdir):
            os.makedirs(testdir)
        sp.run(['cp', os.path.join(bindir, elf + '.elf'), testdir])
        emitTraceScripts(testdir, elf, npts)

        # Get traces
        print('Simulating {}, {} points... '.format(elf, npts),
              end='',
              flush=True)
        simTraceRaw = runTrace(testdir, 'sim')
        with open(os.path.join(testdir, 'sim-trace-raw.log'), 'w+') as of:
            of.write(simTraceRaw)

        # Get measurement from trace file, if it exists, otherwise run on dev board
        if (os.path.exists(os.path.join(idir, elf, 'meas-trace-raw.log'))):
            print('reading measured from file...', end='', flush=True)
            with open(os.path.join(idir, elf, 'meas-trace-raw.log'),
                      'r') as infile:
                measTraceRaw = infile.read()
        else:
            print('measuring trace off dev board...', end='', flush=True)
            measTraceRaw = runTrace(testdir, 'meas')
            with open(os.path.join(indir, elf, 'meas-trace-raw.log'),
                      'w+') as of:
                of.write(measTraceRaw)

        # Parse traces
        simTrace = filterConsecutive(parseTrace(simTraceRaw))
        measTrace = filterConsecutive(parseTrace(measTraceRaw))

        if (len(simTrace) == 0):
            print("Trace parsing failed.")
            exit(1)

        #Find mismatches
        availablePoints = min([len(simTrace), len(measTrace), npts])
        print('verifying {} pts...'.format(availablePoints),
              end='',
              flush=True)
        n_err = 0
        for i in range(availablePoints):
            s = simTrace[i]
            m = measTrace[i]
            if not compareValues(s, m):  #if (s != m):
                anyFailed = True
                n_err += 1
                print('FAIL')
                print('ERROR: Found mismatch: -------------')
                print('ERROR: Found mismatch: -------------', file=sys.stderr)
                print("PC: 0x{:04x}(sim) 0x{:04x}(meas)".format(
                    s['pc'], m['pc']))
                compareValues(s, m)
                print("Previous PC: 0x{:04x}(sim) 0x{:04x}(meas)".format(
                    simTrace[i - 1]['pc'], measTrace[i - 1]['pc']))
                compareValues(simTrace[i - 1], measTrace[i - 1])
                print(s)
                print(m)
                emitReproduceScript(simTrace[i - 1]['pc'], testdir, elf)
                break
        else:
            print('PASS')
    return anyFailed


if __name__ == "__main__":
    main()
