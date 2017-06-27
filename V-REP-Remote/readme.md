# Remote Binding for V-REP in Python

## Notes:
1. The application is written in `app.py`.
2. Each robot is controlled via separate thread, communicating with each other via mail box.
3. The GIL ( Global interpreter lock) may get in our way with multi threading. ( needs to be tested)
4. The control is done on the level of joints and motors, so any differential wheel based robot will do it.

---

## How to setup the binding


* Server side
  * Make sure that `v_repExtRemoteApi.dll` does exist in root directory of V-REP
* Client Side (These files' content varies with V-REP version) 
  * Copy `programming/python/vrep.py` and `programming/python/vrepConst.py` to your project folder.
  * Copy `programming/lib/lib/32Bit/remoteApi.dll` to your project folder.

* Run steps:
  1. Run V-REP and open a new scene.
  2. Just drag and drop the model into the scene to create a new robot.
    * You can copy the model file `DragonOne.ttm` to V-REP models folder
  3. Run `app.py` with python via command line.

### Notes:

* Use V-REP version 3.4 for best compatability.
