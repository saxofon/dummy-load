# dummy-load

Simple process spinning around in a loop, giving a specified load in percentage as arg1.

For example:

dummy-load 24.5

Load regulation is made by a PID regulator, trying to keep the load at the requested load,
often due to other things happen in the system it will be in quite neighbourhood (not exact) though.
