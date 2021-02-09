# Interface for FSSIM

In order to create a universal simulator, FSSIM comunicated with your own framework through this interface.
This package does not contain any logic, it only translated messages from FSSIM into your messages. 

* `config`: contains definitions of topics which are translated

# How to run FSSIM

Detailed description can be found in [fssim](https://github.com/bitfsd/fssim)

### Run FSSIM parallely to your code
If you want to run simulator in one terminal window and be able to test your code in parallel execute
1. `roslaunch fssim_interface only_fssim.launch`
2. Your pipeline, e.g. `roslaunch control trackdrive.launch
