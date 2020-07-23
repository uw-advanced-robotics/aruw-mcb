## Building the CppUTest repository

Enter into the cpputest submodule and run the following:

```bash
$ cd cpputest_build
$ autoreconf .. -i
$ ../configure
$ make
```

Then in `cpputest_build`, `cp ./lib/* ../../lib/cpputest`