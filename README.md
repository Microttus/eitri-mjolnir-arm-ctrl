# eitri-mjolnir-arm-ctrl

A package for reading twist topic, inverse jacobian calculation, 
and serial write of the motor positions

Under resources some required additions can be found:
- Arduino code for excecution can be found in the arduino folder

Comand to launch:

```shell
python ur5 _run_trajectory <ip> --traj <path_file> --z-offset 0.015 --ext-urcap --urcap-port 50001 -v -v
```
