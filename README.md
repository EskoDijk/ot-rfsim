# OpenThread on RF-simulator (OT-NS) platform

This repo contains 'ot-rfsim', an example OpenThread platform driver for simulated OT nodes. A simulated OT node can be
started from an RF simulator, such as [OT-NS](https://github.com/EskoDijk/ot-ns).
It connects to the simulator using the Unix Domain Socket as provided in the commandline parameters.

## Prerequisites

After cloning the repo, you must initialize the git submodule.

```
$ git submodule update --init
```

The submodule is `openthread` with the (latest) openthread main branch code.

## Building

### Build using cmake for use in OT-NS

Below shows an example default build for use of the binaries in OT-NS simulation.

```bash
$ ./script/build
```

Below shows an example build for OT-NS with build option OT_FULL_LOGS set to 'ON',
for extra debug info that can then be optionally displayed in OT-NS using the `watch` command.

```bash
$ ./script/build -DOT_FULL_LOGS=ON
```

After a successful build, the executable files are found in the directory:

```
./build/bin
```

## Running

After building, the executables can be copied to an OT-NS working directory for use in OT-NS simulation. 
This is the preferred way. Also, for a quick test, OT-NS can be started from the binaries directory and 
then the built `ot-cli-ftd` will be used in the simulation:

```bash
$ cd build/bin
$ otns
> add router x 50 y 50
1
Done
```

Also the executables can be tested on the command line as follows:

```bash
$ cd build/bin
$ ./ot-cli-ftd
Usage: ot-cli-ftd <nodeNumber> <OTNS-socket-file>
$
```

This will print a usage message and exit the node. 


## Contributing

We would love for you to contribute to OpenThread and help make it even better than it is today! See our 
[Contributing Guidelines](https://github.com/openthread/openthread/blob/main/CONTRIBUTING.md) for more information.

Contributors are required to abide by our 
[Code of Conduct](https://github.com/openthread/openthread/blob/main/CODE_OF_CONDUCT.md) and 
[Coding Conventions and Style Guide](https://github.com/openthread/openthread/blob/main/STYLE_GUIDE.md).

## License

OpenThread is released under the [BSD 3-Clause license](https://github.com/EskoDijk/ot-rfsim/blob/main/LICENSE). 
See the [`LICENSE`](https://github.com/EskoDijk/ot-rfsim/blob/main/LICENSE) file for more information.

Please only use the OpenThread name and marks when accurately referencing this software distribution. Do not use the 
marks in a way that suggests you are endorsed by or otherwise affiliated with Nest, Google, or The Thread Group.

## Need help?

OpenThread support is available on GitHub:

- Bugs and feature requests pertaining to the ot-rfsim platform — [submit to the ot-rfsim Issue Tracker](https://github.com/EskoDijk/ot-rfsim/issues)
- OpenThread bugs and feature requests — [submit to the OpenThread Issue Tracker](https://github.com/openthread/openthread/issues)
- Community Discussion - [ask questions, share ideas, and engage with other community members](https://github.com/openthread/openthread/discussions)
